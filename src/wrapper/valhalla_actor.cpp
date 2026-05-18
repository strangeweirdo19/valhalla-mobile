#include <boost/property_tree/ptree.hpp>
#include <valhalla/tyr/actor.h>
#include <valhalla/baldr/rapidjson_utils.h>
#include <valhalla/baldr/directededge.h>
#include <valhalla/baldr/edgeinfo.h>
#include <valhalla/baldr/tilehierarchy.h>
#include <valhalla/midgard/tiles.h>
#include <valhalla/midgard/pointll.h>
#include <valhalla/loki/worker.h>
#include "valhalla_actor.h"

#include <rapidjson/document.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>

#include <algorithm>
#include <cstdio>
#include <cstdint>
#include <unordered_set>
#include <tuple>
#include <vector>

namespace {

constexpr double kMinRoutePointSpacingMeters = 5.0;
constexpr double kMaxStructureSnapDistanceMeters = 30.0;

enum class StructureType : uint8_t {
  Bridge = 0,
  Tunnel = 1,
};

static void append_route_shape_simplified(std::vector<valhalla::midgard::PointLL>& route_shape,
                                          const std::vector<valhalla::midgard::PointLL>& shape) {
  if (shape.empty()) {
    return;
  }

  for (const auto& p : shape) {
    if (route_shape.empty()) {
      route_shape.push_back(p);
      continue;
    }
    const auto& last = route_shape.back();
    if (last.Distance(p) >= kMinRoutePointSpacingMeters) {
      route_shape.push_back(p);
    }
  }
}

static bool add_structures_to_json(std::string& json,
                                  const std::vector<std::tuple<StructureType, double, double, bool>>&
                                      structures) {
  rapidjson::Document doc;
  doc.Parse(json.c_str());
  if (doc.HasParseError() || !doc.IsObject()) {
    return false;
  }

  auto& alloc = doc.GetAllocator();
  rapidjson::Value structures_value(rapidjson::kArrayType);

  for (const auto& s : structures) {
    const auto type = std::get<0>(s);
    const auto lat = std::get<1>(s);
    const auto lon = std::get<2>(s);
    const auto take = std::get<3>(s);

    rapidjson::Value obj(rapidjson::kObjectType);
    rapidjson::Value type_value(rapidjson::kStringType);
    if (type == StructureType::Bridge) {
      type_value.SetString("bridge", alloc);
    } else {
      type_value.SetString("tunnel", alloc);
    }
    obj.AddMember("type", type_value, alloc);
    obj.AddMember("lat", lat, alloc);
    obj.AddMember("lon", lon, alloc);
    obj.AddMember("take", take, alloc);

    structures_value.PushBack(obj, alloc);
  }

  if (doc.HasMember("structures")) {
    doc.RemoveMember("structures");
  }
  doc.AddMember("structures", structures_value, alloc);

  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  doc.Accept(writer);
  json.assign(buffer.GetString(), buffer.GetSize());
  return true;
}

static void extract_structures(const valhalla::Api& api,
                               valhalla::baldr::GraphReader& reader,
                               std::vector<std::tuple<StructureType, double, double, bool>>& out) {
  out.clear();

  if (api.trip().routes_size() == 0) {
    return;
  }

  const auto& route = api.trip().routes(0);
  if (route.legs_size() == 0) {
    return;
  }

  std::unordered_set<uint64_t> route_edge_ids;
  route_edge_ids.reserve(4096);

  std::vector<valhalla::midgard::PointLL> route_shape;
  route_shape.reserve(8192);

  std::unordered_set<uint64_t> seen;
  seen.reserve(2048);

  const auto seen_key = [](const uint64_t edge_value, const StructureType type, const bool take) {
    // GraphId uses 46 bits for the non-spare part, leaving high bits free.
    const uint64_t type_bits = static_cast<uint64_t>(type) & 0x3ULL;
    const uint64_t take_bits = take ? 1ULL : 0ULL;
    return edge_value | (type_bits << 62) | (take_bits << 61);
  };

  // Build the on-route edge set + a simplified route polyline.
  for (int leg_idx = 0; leg_idx < route.legs_size(); ++leg_idx) {
    const auto& leg = route.legs(leg_idx);
    for (int node_idx = 0; node_idx < leg.node_size(); ++node_idx) {
      const auto& trip_edge = leg.node(node_idx).edge();
      // TripEdge::id is a GraphId value.
      valhalla::baldr::GraphId edge_id;
      try {
        edge_id = valhalla::baldr::GraphId(trip_edge.id());
      } catch (...) {
        continue;
      }
      if (!edge_id.is_valid()) {
        continue;
      }
      route_edge_ids.insert(edge_id.value);

      valhalla::baldr::graph_tile_ptr edge_tile = nullptr;
      const auto* de = reader.directededge(edge_id, edge_tile);
      if (!de || !edge_tile) {
        continue;
      }
      const auto info = edge_tile->edgeinfo(de);
      const auto& shape = info.shape();
      append_route_shape_simplified(route_shape, shape);

      // Structures that are actually taken (on the route).
      if (de->bridge() || de->tunnel()) {
        const auto mid = shape.empty() ? valhalla::midgard::PointLL{} : shape[shape.size() / 2];
        if (!mid.IsValid()) {
          continue;
        }
        if (de->bridge()) {
          const auto k = seen_key(edge_id.value, StructureType::Bridge, true);
          if (seen.insert(k).second) {
            out.emplace_back(StructureType::Bridge, mid.lat(), mid.lng(), true);
          }
        }
        if (de->tunnel()) {
          const auto k = seen_key(edge_id.value, StructureType::Tunnel, true);
          if (seen.insert(k).second) {
            out.emplace_back(StructureType::Tunnel, mid.lat(), mid.lng(), true);
          }
        }
      }
    }
  }

  if (route_shape.size() < 2) {
    return;
  }

  // Search for nearby bridge/tunnel edges (including not-taken) via max-level edge bins.
  const auto max_level = valhalla::baldr::TileHierarchy::levels().back().level;
  const auto& tiles = valhalla::baldr::TileHierarchy::levels().back().tiles;

  // Determine which (max-level tile, bin) pairs the route geometry passes through.
  const auto route_bins = tiles.Intersect(route_shape);
  std::unordered_set<uint64_t> visited_bins;
  visited_bins.reserve(route_bins.size() * 8);

  for (const auto& tile_and_bins : route_bins) {
    const int32_t max_tile_id = tile_and_bins.first;
    if (max_tile_id < 0) {
      continue;
    }

    const valhalla::baldr::GraphId max_tile_graphid(static_cast<uint32_t>(max_tile_id), max_level, 0);
    auto max_tile = reader.GetGraphTile(max_tile_graphid);
    if (!max_tile) {
      continue;
    }

    for (const auto bin_id : tile_and_bins.second) {
      const uint64_t bin_key = (static_cast<uint64_t>(static_cast<uint32_t>(max_tile_id)) << 16) |
                               static_cast<uint64_t>(bin_id);
      if (!visited_bins.insert(bin_key).second) {
        continue;
      }

      const auto candidates = max_tile->GetBin(static_cast<size_t>(bin_id));
      for (const auto& candidate_edge_id : candidates) {
        if (!candidate_edge_id.is_valid()) {
          continue;
        }

        valhalla::baldr::graph_tile_ptr edge_tile = nullptr;
        const auto* de = reader.directededge(candidate_edge_id, edge_tile);
        if (!de || !edge_tile) {
          continue;
        }

        if (!de->bridge() && !de->tunnel()) {
          continue;
        }

        const auto info = edge_tile->edgeinfo(de);
        const auto& shape = info.shape();
        if (shape.empty()) {
          continue;
        }
        const auto mid = shape[shape.size() / 2];
        if (!mid.IsValid()) {
          continue;
        }

        // Keep only structures that are actually close to the route geometry.
        const auto closest = mid.ClosestPoint(route_shape);
        const auto dist_m = std::get<1>(closest);
        if (dist_m > kMaxStructureSnapDistanceMeters) {
          continue;
        }

        const bool take = route_edge_ids.find(candidate_edge_id.value) != route_edge_ids.end();
        if (de->bridge()) {
          const auto k = seen_key(candidate_edge_id.value, StructureType::Bridge, take);
          if (seen.insert(k).second) {
            out.emplace_back(StructureType::Bridge, mid.lat(), mid.lng(), take);
          }
        }
        if (de->tunnel()) {
          const auto k = seen_key(candidate_edge_id.value, StructureType::Tunnel, take);
          if (seen.insert(k).second) {
            out.emplace_back(StructureType::Tunnel, mid.lat(), mid.lng(), take);
          }
        }
      }
    }
  }

  // Stable ordering: type, take, then lat/lon.
  std::sort(out.begin(), out.end(), [](const auto& a, const auto& b) {
    if (std::get<0>(a) != std::get<0>(b)) {
      return std::get<0>(a) < std::get<0>(b);
    }
    if (std::get<3>(a) != std::get<3>(b)) {
      return std::get<3>(a) < std::get<3>(b);
    }
    if (std::get<1>(a) != std::get<1>(b)) {
      return std::get<1>(a) < std::get<1>(b);
    }
    return std::get<2>(a) < std::get<2>(b);
  });
}

} // namespace

class TileGetterWrapper : public valhalla::baldr::tile_getter_t {
public:
  /**
   * @param pool_size  the number of curler instances in the pool
   * @param user_agent  user agent to use by curlers for HTTP requests
   * @param gzipped  whether to request for gzip compressed data
   * @param user_pw  the "user:pwd" for HTTP basic auth
   */
  TileGetterWrapper(ValhallaMobileHttpClient* http_client, bool is_gzipped): http_client(http_client), is_gzipped(is_gzipped) {
  }

  GET_response_t get(const std::string& url,
                     const uint64_t range_offset = 0,
                     const uint64_t range_size = 0) override {
    GET_response_t result;
    if (http_client) { 
        result = http_client->get(url, range_offset, range_size);
    } else {
        result.status_ = tile_getter_t::status_code_t::FAILURE;
    }
    return result;
  }

  HEAD_response_t head(const std::string& url, header_mask_t header_mask) override {
    HEAD_response_t result;
    if (http_client) { 
        result = http_client->head(url, header_mask);
    } else {
        result.status_ = tile_getter_t::status_code_t::FAILURE;
    }
    return result;
  }

  bool gzipped() const override {
    return is_gzipped;
  }

  ~TileGetterWrapper() {
    delete http_client;
  };

private:
  bool is_gzipped;
  ValhallaMobileHttpClient* http_client;
};


ValhallaActor::ValhallaActor(const std::string& config_path, ValhallaMobileHttpClient* http_client) {
std::string config_file(config_path);
    
    // Set up the config object
    boost::property_tree::ptree config;
    rapidjson::read_json(config_file, config);

    auto mjolnir_config = config.get_child("mjolnir");
    graph_reader = std::make_unique<valhalla::baldr::GraphReader>(
      mjolnir_config, 
      std::make_unique<TileGetterWrapper>(http_client, mjolnir_config.get<bool>("tile_url_gz", false))
    );
    // Setup the actor
    actor = std::make_unique<valhalla::tyr::actor_t>(config, *graph_reader, true);
}

std::string ValhallaActor::route(const std::string& request) {
    // Convert the request to a std::string
    std::string req = std::string(request);
    
    // Produce the route result and capture the low-level API output so we can
    // reliably extract bridge/tunnel locations from the underlying graph.
    valhalla::Api api;
    std::string result = actor->route(req, nullptr, &api);

    // Best-effort: add a top-level `structures` array when the output is JSON.
    try {
      std::vector<std::tuple<StructureType, double, double, bool>> structures;
      extract_structures(api, *graph_reader, structures);
      add_structures_to_json(result, structures);
    } catch (const std::exception& e) {
      printf("[ValhallaActor] structures extraction std::exception: %s\n", e.what());
    } catch (...) {
      printf("[ValhallaActor] structures extraction unknown exception\n");
    }

    return result;
}
