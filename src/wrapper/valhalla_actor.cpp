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
#include <array>
#include <climits>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <unordered_set>
#include <tuple>
#include <vector>

namespace {

constexpr double kMinRoutePointSpacingMeters = 5.0;
constexpr double kMaxStructureSnapDistanceMeters = 30.0;
constexpr double kMaxUnconnectedStructureSnapDistanceMeters = 10.0;
constexpr double kOppositeDirectionHeadingDiffDegrees = 90.0;

// A structure is "straight-ahead" if its heading is within this many degrees
// of the route's pre-turn approach direction.
constexpr double kStraightAheadAlignmentDegrees = 45.0;

// The route must change direction by at least this much near the snap point
// for the divergence to count as a real turn.  20° catches gentle bends on
// Indian service-road layouts that the previous 30° threshold missed.
constexpr double kTurnDetectionDegrees = 20.0;

// For unconnected bridges, the bearing from the route snap point TO the bridge
// entry must be within this many degrees of the bridge's own travel heading.
// This distinguishes a bridge that starts ahead of the route (parallel
// carriageway offset: ~0° bearing diff) from one that is beside it
// (perpendicular carriageway offset: ~90° bearing diff).
constexpr double kBridgeEntryBearingAlignmentDegrees = 50.0;

// A bridge that subtends this angle or more relative to the route is treated
// as a true overpass/underpass crossing rather than a parallel structure.
constexpr double kCrossingStructureMinAngleDegrees = 50.0;

constexpr size_t kMaxAlternateConnectors = 20;
constexpr size_t kMaxConnectorEdges = 3;
constexpr double kMaxConnectorDistanceMeters = 200.0;

enum class StructureType : uint8_t {
  Bridge = 0,
  Tunnel = 1,
};

struct RouteConnector {
  valhalla::midgard::PointLL entry;
  valhalla::midgard::PointLL exit;
  std::vector<valhalla::midgard::PointLL> shape;
  int entry_segment_index = -1;
};

static void append_route_shape_simplified(std::vector<valhalla::midgard::PointLL>& route_shape,
                                          const std::vector<valhalla::midgard::PointLL>& shape,
                                          const bool forward) {
  if (shape.empty()) {
    return;
  }

  if (forward) {
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
    return;
  }

  for (auto it = shape.rbegin(); it != shape.rend(); ++it) {
    const auto& p = *it;
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

static valhalla::midgard::PointLL structure_entry_point(const std::vector<valhalla::midgard::PointLL>& shape,
                                                       const bool forward) {
  if (shape.empty()) {
    return {};
  }
  return forward ? shape.front() : shape.back();
}

static double structure_travel_heading(const std::vector<valhalla::midgard::PointLL>& shape,
                                       const bool forward) {
  if (shape.size() < 2) {
    return 0.0;
  }
  if (forward) {
    return shape.front().Heading(shape[1]);
  }
  return shape.back().Heading(shape[shape.size() - 2]);
}

static double heading_diff_degrees(const double a, const double b) {
  double diff = std::fabs(a - b);
  diff = std::fmod(diff, 360.0);
  if (diff > 180.0) {
    diff = 360.0 - diff;
  }
  return diff;
}

static bool is_opposite_direction(const std::vector<valhalla::midgard::PointLL>& route_shape,
                                  const int route_segment_index,
                                  const std::vector<valhalla::midgard::PointLL>& edge_shape,
                                  const bool forward) {
  if (route_segment_index < 0) {
    return false;
  }

  const size_t route_idx0 = static_cast<size_t>(route_segment_index);
  const size_t route_idx1 = route_idx0 + 1;
  if (route_idx1 >= route_shape.size()) {
    return false;
  }
  if (edge_shape.size() < 2) {
    return false;
  }

  const auto route_heading = route_shape[route_idx0].Heading(route_shape[route_idx1]);
  const auto edge_heading = structure_travel_heading(edge_shape, forward);
  return heading_diff_degrees(route_heading, edge_heading) > kOppositeDirectionHeadingDiffDegrees;
}

static bool is_point_near_route(const valhalla::midgard::PointLL& point,
                                const std::vector<valhalla::midgard::PointLL>& route_shape,
                                const double max_snap_distance_meters) {
  if (!point.IsValid() || route_shape.size() < 2) {
    return false;
  }
  const auto closest = point.ClosestPoint(route_shape);
  const auto dist_m = std::get<1>(closest);
  return dist_m <= max_snap_distance_meters;
}

static bool is_entry_point_near_route(const valhalla::midgard::PointLL& entry_point,
                                      const std::vector<valhalla::midgard::PointLL>& route_shape,
                                      const double max_snap_distance_meters = kMaxStructureSnapDistanceMeters) {
  return is_point_near_route(entry_point, route_shape, max_snap_distance_meters);
}

static valhalla::midgard::PointLL structure_center_point(const std::vector<valhalla::midgard::PointLL>& shape) {
  if (shape.empty()) {
    return {};
  }

  double lat_sum = 0.0;
  double lon_sum = 0.0;
  for (const auto& p : shape) {
    lat_sum += p.lat();
    lon_sum += p.lng();
  }

  const double count = static_cast<double>(shape.size());
  return {lat_sum / count, lon_sum / count};
}

static bool is_structure_center_near_route(const std::vector<valhalla::midgard::PointLL>& shape,
                                           const std::vector<valhalla::midgard::PointLL>& route_shape,
                                           const double max_snap_distance_meters = kMaxStructureSnapDistanceMeters) {
  const auto center = structure_center_point(shape);
  return is_point_near_route(center, route_shape, max_snap_distance_meters);
}

static uint64_t structure_coord_key(const double lat, const double lon) {
  const auto qlat = static_cast<int32_t>(std::llround(lat * 1e6));
  const auto qlon = static_cast<int32_t>(std::llround(lon * 1e6));
  return (static_cast<uint64_t>(static_cast<uint32_t>(qlat)) << 32) |
         static_cast<uint64_t>(static_cast<uint32_t>(qlon));
}

// Returns true when the bridge/tunnel is the road the route would have taken
// if it had continued straight instead of turning.
//
// Searches a ±2 segment window around snap_seg for a consecutive pair
// [i→i+1→i+2] where:
//   1. heading_diff(h_before, h_after) > kTurnDetectionDegrees  (real turn at i+1)
//   2. heading_diff(bridge_heading, h_before) < kStraightAheadAlignmentDegrees
//
// Window of ±2 (vs. the previous ±1) catches cases where ClosestPoint lands
// one or two segments away from the actual divergence vertex.
// kTurnDetectionDegrees = 20° (vs. previous 30°) catches gentle bends on
// service-road layouts common in South Asia.
//
// Correctly rejects:
//   - Bridges at straight-through junctions (no turn ≥ 20°)
//   - End-of-taken-bridge D→B edges (heading ≈ 180° from approach)
//   - Parallel opposite-direction carriageways (heading ≈ 180° from pre-turn)
static bool is_straight_ahead_avoided_structure(
    const std::vector<valhalla::midgard::PointLL>& route_shape,
    const int snap_seg,
    const std::vector<valhalla::midgard::PointLL>& edge_shape,
    const bool forward) {

  if (snap_seg < 0 || route_shape.size() < 3) {
    return false;
  }

  const double bridge_heading = structure_travel_heading(edge_shape, forward);
  const size_t n = route_shape.size();

  // Window ±2 around snap_seg.
  const size_t lo = (static_cast<size_t>(snap_seg) > 2)
                        ? static_cast<size_t>(snap_seg) - 2
                        : size_t{0};
  const size_t hi = std::min(static_cast<size_t>(snap_seg) + 2,
                             n > 2 ? n - 2 : size_t{0});

  for (size_t i = lo; i <= hi; ++i) {
    if (i + 2 >= n) {
      break;
    }

    const double h_before = route_shape[i].Heading(route_shape[i + 1]);
    const double h_after  = route_shape[i + 1].Heading(route_shape[i + 2]);

    // Condition 1: meaningful turn at vertex i+1.
    if (heading_diff_degrees(h_before, h_after) < kTurnDetectionDegrees) {
      continue;
    }

    // Condition 2: bridge aligns with the pre-turn approach direction.
    if (heading_diff_degrees(bridge_heading, h_before) < kStraightAheadAlignmentDegrees) {
      return true;
    }
  }

  return false;
}

// Returns true if the bridge entry lies roughly AHEAD of the route snap point
// in the bridge's own travel direction.
//
// This is the key filter for parallel divided-highway carriageways.
//
// Scenario: route diverges at junction J heading northeast.  The flyover has
// two carriageways, both heading northeast:
//   - LEFT carriageway: starts at J (connected, handled separately)
//   - RIGHT carriageway: starts at J_right, ~12m to the right of J
//
// For the RIGHT carriageway:
//   snap_point ≈ J,  bridge_entry = J_right  (~12m to the east of J)
//   bearing_from_J_to_J_right ≈ east (~90°)
//   bridge_heading ≈ northeast (~45°)
//   heading_diff(east, northeast) ≈ 45° → NOT < 50° → filtered ✓
//
// For a bridge that starts 15m AHEAD on the original travel direction:
//   bearing_from_J_to_M ≈ northeast
//   bridge_heading ≈ northeast
//   heading_diff ≈ 0° → passes ✓
//
// Safe-fallthrough: if snap_point and bridge_entry are essentially the same
// point (distance < 2m), the bearing is undefined and we allow the structure.
static bool is_bridge_entry_ahead_of_route(
    const valhalla::midgard::PointLL& route_snap_point,
    const valhalla::midgard::PointLL& bridge_entry,
    const std::vector<valhalla::midgard::PointLL>& edge_shape,
    const bool forward) {

  if (!route_snap_point.IsValid() || !bridge_entry.IsValid()) {
    return true;
  }
  const double dist = route_snap_point.Distance(bridge_entry);
  if (dist < 2.0) {
    return true;  // essentially coincident; allow
  }

  const double bearing_to_entry = route_snap_point.Heading(bridge_entry);
  const double bridge_heading   = structure_travel_heading(edge_shape, forward);
  return heading_diff_degrees(bearing_to_entry, bridge_heading) < kBridgeEntryBearingAlignmentDegrees;
}

// Returns true when the structure crosses the route at a significant angle,
// indicating a true overpass or underpass rather than a parallel structure.
//
// Used to separate the tight-proximity (entry + centre ≤ 10m) case into:
//   - Crossing (bridge ⊥ route, diff > 50°): genuine over/under pass
//   - Parallel (bridge ∥ route, diff ≤ 50°): same corridor, apply turn check
static bool is_crossing_structure(const std::vector<valhalla::midgard::PointLL>& route_shape,
                                  const int snap_seg,
                                  const std::vector<valhalla::midgard::PointLL>& edge_shape,
                                  const bool forward) {
  if (snap_seg < 0) {
    return false;
  }
  const size_t idx0 = static_cast<size_t>(snap_seg);
  if (idx0 + 1 >= route_shape.size()) {
    return false;
  }
  const double route_heading  = route_shape[idx0].Heading(route_shape[idx0 + 1]);
  const double bridge_heading = structure_travel_heading(edge_shape, forward);
  return heading_diff_degrees(route_heading, bridge_heading) > kCrossingStructureMinAngleDegrees;
}

static rapidjson::Value serialize_structures(const std::vector<std::tuple<StructureType, double, double, bool>>& structures,
                                             rapidjson::Document::AllocatorType& alloc) {
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
  return structures_value;
}

static rapidjson::Value serialize_connectors(const std::vector<RouteConnector>& connectors,
                                             rapidjson::Document::AllocatorType& alloc) {
  rapidjson::Value connectors_value(rapidjson::kArrayType);
  for (const auto& c : connectors) {
    rapidjson::Value obj(rapidjson::kObjectType);

    rapidjson::Value entry(rapidjson::kObjectType);
    entry.AddMember("lat", c.entry.lat(), alloc);
    entry.AddMember("lon", c.entry.lng(), alloc);
    obj.AddMember("entry", entry, alloc);

    rapidjson::Value exit_obj(rapidjson::kObjectType);
    exit_obj.AddMember("lat", c.exit.lat(), alloc);
    exit_obj.AddMember("lon", c.exit.lng(), alloc);
    obj.AddMember("exit", exit_obj, alloc);

    rapidjson::Value shape_value(rapidjson::kArrayType);
    shape_value.Reserve(static_cast<rapidjson::SizeType>(c.shape.size()), alloc);
    for (const auto& p : c.shape) {
      rapidjson::Value coord(rapidjson::kArrayType);
      coord.Reserve(2, alloc);
      coord.PushBack(p.lat(), alloc);
      coord.PushBack(p.lng(), alloc);
      shape_value.PushBack(coord, alloc);
    }
    obj.AddMember("shape", shape_value, alloc);

    connectors_value.PushBack(obj, alloc);
  }
  return connectors_value;
}

// Always parses and re-serialises so "version" is injected on every response.
static bool add_structures_to_json(std::string& json,
                                   const std::vector<std::vector<std::tuple<StructureType, double, double, bool>>>&
                                       all_routes_structures,
                                   const std::vector<std::vector<RouteConnector>>&
                                       all_routes_connectors) {
  rapidjson::Document doc;
  doc.Parse(json.c_str());
  if (doc.HasParseError() || !doc.IsObject()) {
    return false;
  }

  auto& alloc = doc.GetAllocator();

  // -- version tag -----------------------------------------------------------
  if (doc.HasMember("version")) {
    doc.RemoveMember("version");
  }
  doc.AddMember("version", rapidjson::Value("v1.0", alloc), alloc);

  // -- structures for the primary route --------------------------------------
  if (!all_routes_structures.empty() && doc.HasMember("trip") && doc["trip"].IsObject()) {
    auto& trip = doc["trip"];
    if (trip.HasMember("structures")) {
      trip.RemoveMember("structures");
    }
    trip.AddMember("structures",
                   serialize_structures(all_routes_structures[0], alloc),
                   alloc);
  }

  // -- structures / connectors for alternates --------------------------------
  const char* alternate_keys[] = {"alternates", "alternatives"};
  for (const auto* key : alternate_keys) {
    if (doc.HasMember(key) && doc[key].IsArray()) {
      auto alternates_array = doc[key].GetArray();
      for (rapidjson::SizeType i = 0; i < alternates_array.Size(); ++i) {
        const size_t route_index = static_cast<size_t>(i + 1);
        auto& alt = alternates_array[i];
        if (!alt.IsObject()) {
          continue;
        }

        if (route_index < all_routes_structures.size()) {
          if (alt.HasMember("trip") && alt["trip"].IsObject()) {
            auto& alt_trip = alt["trip"];
            if (alt_trip.HasMember("structures")) {
              alt_trip.RemoveMember("structures");
            }
            alt_trip.AddMember("structures",
                               serialize_structures(all_routes_structures[route_index], alloc),
                               alloc);
          }
        }

        if (route_index < all_routes_connectors.size() && !all_routes_connectors[route_index].empty()) {
          if (alt.HasMember("connectors")) {
            alt.RemoveMember("connectors");
          }
          alt.AddMember("connectors",
                        serialize_connectors(all_routes_connectors[route_index], alloc),
                        alloc);
        }
      }
    }
  }

  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  doc.Accept(writer);
  json.assign(buffer.GetString(), buffer.GetSize());
  return true;
}

static void extract_structures(const valhalla::Api& api,
                               valhalla::baldr::GraphReader& reader,
                               size_t route_index,
                               std::vector<std::tuple<StructureType, double, double, bool>>& out) {
  out.clear();

  if (api.trip().routes_size() <= route_index) {
    return;
  }

  const auto& route = api.trip().routes(route_index);
  if (route.legs_size() == 0) {
    return;
  }

  std::unordered_set<uint64_t> route_edge_ids;
  route_edge_ids.reserve(4096);

  // All graph nodes touched by the route.  O(1) lookup in Phase 2.
  std::unordered_set<uint64_t> route_node_ids;
  route_node_ids.reserve(4096);

  std::vector<valhalla::midgard::PointLL> route_shape;
  route_shape.reserve(8192);

  std::unordered_set<uint64_t> seen;
  seen.reserve(2048);

  const auto seen_key = [](const uint64_t edge_value, const StructureType type, const bool take) {
    const uint64_t type_bits = static_cast<uint64_t>(type) & 0x3ULL;
    const uint64_t take_bits = take ? 1ULL : 0ULL;
    return edge_value | (type_bits << 62) | (take_bits << 61);
  };

  // -- Phase 1: on-route edge/node sets + simplified route polyline ----------
  for (int leg_idx = 0; leg_idx < route.legs_size(); ++leg_idx) {
    const auto& leg = route.legs(leg_idx);
    for (int node_idx = 0; node_idx < leg.node_size(); ++node_idx) {
      const auto& trip_edge = leg.node(node_idx).edge();
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

      try {
        valhalla::baldr::graph_tile_ptr edge_tile = nullptr;
        const auto* de = reader.directededge(edge_id, edge_tile);
        if (!de || !edge_tile) {
          continue;
        }

        const auto nodes = reader.GetDirectedEdgeNodes(edge_id, edge_tile);
        if (nodes.first.is_valid())  route_node_ids.insert(nodes.first.value);
        if (nodes.second.is_valid()) route_node_ids.insert(nodes.second.value);

        const auto info = edge_tile->edgeinfo(de);
        const auto& shape = info.shape();
        append_route_shape_simplified(route_shape, shape, trip_edge.forward());

        // Structures actually taken.
        if (de->bridge() || de->tunnel()) {
          const auto entry = structure_entry_point(shape, trip_edge.forward());
          if (!entry.IsValid()) {
            continue;
          }
          if (de->bridge()) {
            const auto k = seen_key(edge_id.value, StructureType::Bridge, true);
            if (seen.insert(k).second) {
              out.emplace_back(StructureType::Bridge, entry.lat(), entry.lng(), true);
            }
          }
          if (de->tunnel()) {
            const auto k = seen_key(edge_id.value, StructureType::Tunnel, true);
            if (seen.insert(k).second) {
              out.emplace_back(StructureType::Tunnel, entry.lat(), entry.lng(), true);
            }
          }
        }
      } catch (const std::exception& e) {
        printf("[ValhallaActor] Loop 1 edge_id: %llu exception: %s\n", edge_id.value, e.what());
      }
    }
  }

  if (route_shape.size() < 2) {
    return;
  }

  // -- Phase 2: nearby bridge/tunnel edges via max-level tile bins -----------
  uint8_t max_level = 2;
  try {
    max_level = valhalla::baldr::TileHierarchy::levels().back().level;
  } catch (const std::exception& e) {
    printf("[ValhallaActor] TileHierarchy max_level exception: %s\n", e.what());
    return;
  }

  try {
    const auto& tiles = valhalla::baldr::TileHierarchy::levels().back().tiles;

    const auto route_bins = tiles.Intersect(route_shape);
    std::unordered_set<uint64_t> visited_bins;
    visited_bins.reserve(route_bins.size() * 8);

    for (const auto& tile_and_bins : route_bins) {
      const int32_t max_tile_id = tile_and_bins.first;
      if (max_tile_id < 0) {
        continue;
      }

      const valhalla::baldr::GraphId max_tile_graphid(static_cast<uint32_t>(max_tile_id), max_level, 0);
      valhalla::baldr::graph_tile_ptr max_tile = nullptr;
      try {
        max_tile = reader.GetGraphTile(max_tile_graphid);
      } catch (const std::exception& e) {
        printf("[ValhallaActor] GetGraphTile tile_id: %d exception: %s\n", max_tile_id, e.what());
        continue;
      }
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

          try {
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
            const auto entry = structure_entry_point(shape, de->forward());
            if (!entry.IsValid()) {
              continue;
            }

            const bool take = route_edge_ids.find(candidate_edge_id.value) != route_edge_ids.end();

            // Node-based connectivity: O(1), no extra tile I/O, level-agnostic.
            const auto cand_nodes = reader.GetDirectedEdgeNodes(candidate_edge_id, edge_tile);
            const bool connected_to_route =
                (cand_nodes.first.is_valid()  && route_node_ids.count(cand_nodes.first.value)  > 0) ||
                (cand_nodes.second.is_valid() && route_node_ids.count(cand_nodes.second.value) > 0);

            // Snap entry to route once; snap_point and entry_dist used below.
            const auto closest       = entry.ClosestPoint(route_shape);
            const auto& snap_point   = std::get<0>(closest);
            const double entry_dist_m = std::get<1>(closest);
            const int    snap_seg     = std::get<2>(closest);

            // ── GATE LOGIC ─────────────────────────────────────────────────
            if (!take) {

              if (connected_to_route) {
                // ── CONNECTED BRIDGE ──────────────────────────────────────
                // The bridge shares a graph node with the route.
                // Only keep if the route genuinely turned away from it.
                // No bearing check needed: the entry IS at (or very near)
                // a route node, so the snap-to-entry bearing is degenerate.
                if (!is_straight_ahead_avoided_structure(route_shape, snap_seg,
                                                         shape, de->forward())) {
                  continue;
                }

              } else {
                // ── UNCONNECTED BRIDGE ────────────────────────────────────
                // Hard outer limit: ignore anything farther than 30 m.
                if (entry_dist_m > kMaxStructureSnapDistanceMeters) {
                  continue;
                }

                const bool entry_tight =
                    entry_dist_m <= kMaxUnconnectedStructureSnapDistanceMeters;
                const bool center_tight =
                    is_structure_center_near_route(shape, route_shape,
                                                   kMaxUnconnectedStructureSnapDistanceMeters);

                if (entry_tight && center_tight) {
                  // Both entry and centre are ≤ 10 m: could be overpass/underpass
                  // OR a very close parallel carriageway.  Distinguish by angle.
                  if (is_crossing_structure(route_shape, snap_seg, shape, de->forward())) {
                    // True crossing overpass/underpass (bridge at ≥ 50° to route).
                    // Reject only the exact opposite direction (other carriageway
                    // of a two-way bridge going the opposite way).
                    if (is_opposite_direction(route_shape, snap_seg, shape, de->forward())) {
                      continue;
                    }
                  } else {
                    // Parallel structure (bridge ∥ route, angle < 50°).
                    // Apply the full stay-on-ground test + bearing check.
                    if (!is_straight_ahead_avoided_structure(route_shape, snap_seg,
                                                             shape, de->forward())) {
                      continue;
                    }
                    if (!is_bridge_entry_ahead_of_route(snap_point, entry,
                                                        shape, de->forward())) {
                      continue;
                    }
                  }

                } else {
                  // Entry OR centre is beyond 10 m (typical: flyover that starts
                  // just past the divergence junction, or wide divided highway).
                  //
                  // stay-on-ground test: did the route turn away from this road?
                  if (!is_straight_ahead_avoided_structure(route_shape, snap_seg,
                                                           shape, de->forward())) {
                    continue;
                  }
                  // Parallel-carriageway filter: the direction from the route
                  // snap point to the bridge entry must be roughly aligned with
                  // the bridge's travel direction.  Rejects right-side carriageways
                  // whose entry is offset sideways from the divergence junction
                  // (~90° bearing mismatch) while keeping straight-ahead bridges
                  // starting 10-30 m ahead (~0° bearing mismatch).
                  if (!is_bridge_entry_ahead_of_route(snap_point, entry,
                                                      shape, de->forward())) {
                    continue;
                  }
                }
              }
            }
            // ── END GATE ───────────────────────────────────────────────────

            if (de->bridge()) {
              const auto k = seen_key(candidate_edge_id.value, StructureType::Bridge, take);
              if (seen.insert(k).second) {
                out.emplace_back(StructureType::Bridge, entry.lat(), entry.lng(), take);
              }
            }
            if (de->tunnel()) {
              const auto k = seen_key(candidate_edge_id.value, StructureType::Tunnel, take);
              if (seen.insert(k).second) {
                out.emplace_back(StructureType::Tunnel, entry.lat(), entry.lng(), take);
              }
            }
          } catch (const std::exception& e) {
            printf("[ValhallaActor] Loop 2 candidate edge_id: %llu exception: %s\n", candidate_edge_id.value, e.what());
          }
        }
      }
    }
  } catch (const std::exception& e) {
    printf("[ValhallaActor] Nearby search general exception: %s\n", e.what());
  }

  // -- Deduplicate: drop take=false that share coords with take=true ---------
  std::unordered_map<uint64_t, std::array<bool, 2>> take_by_coord;
  take_by_coord.reserve(out.size());
  for (const auto& s : out) {
    if (!std::get<3>(s)) {
      continue;
    }
    const auto key = structure_coord_key(std::get<1>(s), std::get<2>(s));
    take_by_coord[key][static_cast<size_t>(std::get<0>(s))] = true;
  }

  if (!take_by_coord.empty()) {
    std::vector<std::tuple<StructureType, double, double, bool>> filtered;
    filtered.reserve(out.size());
    for (const auto& s : out) {
      if (!std::get<3>(s)) {
        const auto key = structure_coord_key(std::get<1>(s), std::get<2>(s));
        const auto it = take_by_coord.find(key);
        if (it != take_by_coord.end() && it->second[static_cast<size_t>(std::get<0>(s))]) {
          continue;
        }
      }
      filtered.push_back(s);
    }
    out.swap(filtered);
  }

  // -- Stable sort: type, take, lat, lon -------------------------------------
  try {
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
  } catch (const std::exception& e) {
    printf("[ValhallaActor] Sorting exception: %s\n", e.what());
  }
}

static void collect_route_graph(const valhalla::Api& api,
                                valhalla::baldr::GraphReader& reader,
                                size_t route_index,
                                std::unordered_set<uint64_t>& edge_ids,
                                std::unordered_set<uint64_t>& node_ids,
                                std::vector<valhalla::midgard::PointLL>* route_shape) {
  edge_ids.clear();
  node_ids.clear();
  if (route_shape) {
    route_shape->clear();
  }

  if (api.trip().routes_size() <= route_index) {
    return;
  }

  const auto& route = api.trip().routes(route_index);
  if (route.legs_size() == 0) {
    return;
  }

  for (int leg_idx = 0; leg_idx < route.legs_size(); ++leg_idx) {
    const auto& leg = route.legs(leg_idx);
    for (int node_idx = 0; node_idx < leg.node_size(); ++node_idx) {
      const auto& trip_node = leg.node(node_idx);
      if (!trip_node.has_edge()) {
        continue;
      }
      const auto& trip_edge = trip_node.edge();

      valhalla::baldr::GraphId edge_id;
      try {
        edge_id = valhalla::baldr::GraphId(trip_edge.id());
      } catch (...) {
        continue;
      }
      if (!edge_id.is_valid()) {
        continue;
      }

      edge_ids.insert(edge_id.value);

      valhalla::baldr::graph_tile_ptr edge_tile = nullptr;
      const auto* de = reader.directededge(edge_id, edge_tile);
      if (!de || !edge_tile) {
        continue;
      }

      const auto nodes = reader.GetDirectedEdgeNodes(edge_id, edge_tile);
      if (nodes.first.is_valid()) {
        node_ids.insert(nodes.first.value);
      }
      if (nodes.second.is_valid()) {
        node_ids.insert(nodes.second.value);
      }

      if (route_shape) {
        const auto info = edge_tile->edgeinfo(de);
        const auto& shape = info.shape();
        append_route_shape_simplified(*route_shape, shape, trip_edge.forward());
      }
    }
  }
}

static void collect_connectors(const std::unordered_set<uint64_t>& main_node_ids,
                               const std::unordered_set<uint64_t>& main_edge_ids,
                               const std::unordered_set<uint64_t>& alt_node_ids,
                               const std::unordered_set<uint64_t>& alt_edge_ids,
                               valhalla::baldr::GraphReader& reader,
                               const std::vector<valhalla::midgard::PointLL>& main_route_shape,
                               std::vector<RouteConnector>& out) {
  out.clear();
  std::unordered_set<uint64_t> seen_connectors;
  seen_connectors.reserve(kMaxAlternateConnectors * 2);

  struct QueueItem {
    valhalla::baldr::GraphId node;
    std::vector<valhalla::baldr::GraphId> edges;
    double distance_m = 0.0;
  };

  auto add_edge_shape = [&reader](const valhalla::baldr::GraphId& edge_id,
                                  std::vector<valhalla::midgard::PointLL>& shape_out) {
    valhalla::baldr::graph_tile_ptr edge_tile = nullptr;
    const auto* de = reader.directededge(edge_id, edge_tile);
    if (!de || !edge_tile) {
      return;
    }
    auto shape = edge_tile->edgeinfo(de).shape();
    if (!de->forward()) {
      std::reverse(shape.begin(), shape.end());
    }
    if (shape_out.empty()) {
      shape_out.insert(shape_out.end(), shape.begin(), shape.end());
      return;
    }
    if (!shape.empty()) {
      shape_out.insert(shape_out.end(), std::next(shape.begin()), shape.end());
    }
  };

  auto connector_key = [](uint64_t entry_node, uint64_t exit_node) {
    uint64_t seed = entry_node + 0x9e3779b97f4a7c15ULL;
    seed ^= exit_node + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
    return seed;
  };

  for (const auto node_value : main_node_ids) {
    if (out.size() >= kMaxAlternateConnectors) {
      break;
    }

    valhalla::baldr::GraphId start_node(node_value);
    valhalla::baldr::graph_tile_ptr start_tile = nullptr;
    const auto* start_nodeinfo = reader.nodeinfo(start_node, start_tile);
    if (!start_nodeinfo || !start_tile) {
      continue;
    }

    std::vector<QueueItem> queue;
    queue.reserve(32);
    queue.push_back({start_node, {}, 0.0});

    std::unordered_map<uint64_t, std::pair<size_t, double>> best;
    best[start_node.value] = {0, 0.0};

    for (size_t qi = 0; qi < queue.size(); ++qi) {
      const auto current = queue[qi];
      if (current.edges.size() >= kMaxConnectorEdges) {
        continue;
      }

      valhalla::baldr::graph_tile_ptr node_tile = nullptr;
      const auto* nodeinfo = reader.nodeinfo(current.node, node_tile);
      if (!nodeinfo || !node_tile) {
        continue;
      }

      valhalla::baldr::GraphId edge_id(current.node.tileid(), current.node.level(), nodeinfo->edge_index());
      const auto* de = node_tile->directededge(nodeinfo->edge_index());
      for (uint32_t i = 0; i < nodeinfo->edge_count(); ++i, ++de, ++edge_id) {
        if (!de || de->is_shortcut() || de->IsTransitLine()) {
          continue;
        }
        if (main_edge_ids.find(edge_id.value) != main_edge_ids.end()) {
          continue;
        }
        if (alt_edge_ids.find(edge_id.value) != alt_edge_ids.end()) {
          continue;
        }

        const double next_distance = current.distance_m + static_cast<double>(de->length());
        if (next_distance > kMaxConnectorDistanceMeters) {
          continue;
        }

        const auto endnode = de->endnode();
        if (!endnode.is_valid()) {
          continue;
        }

        std::vector<valhalla::baldr::GraphId> next_edges = current.edges;
        next_edges.push_back(edge_id);

        if (alt_node_ids.find(endnode.value) != alt_node_ids.end()) {
          const auto key = connector_key(start_node.value, endnode.value);
          if (!seen_connectors.insert(key).second) {
            continue;
          }

          valhalla::baldr::graph_tile_ptr end_tile = nullptr;
          const auto* end_nodeinfo = reader.nodeinfo(endnode, end_tile);
          if (!end_nodeinfo || !end_tile) {
            continue;
          }

          RouteConnector connector;
          connector.entry = start_nodeinfo->latlng(start_tile->header()->base_ll());
          connector.exit = end_nodeinfo->latlng(end_tile->header()->base_ll());
          for (const auto& edge : next_edges) {
            add_edge_shape(edge, connector.shape);
          }

          if (main_route_shape.size() >= 2 && connector.entry.IsValid()) {
            const auto closest = connector.entry.ClosestPoint(main_route_shape);
            connector.entry_segment_index = std::get<2>(closest);
          }

          out.push_back(std::move(connector));
          if (out.size() >= kMaxAlternateConnectors) {
            break;
          }
          continue;
        }

        const size_t next_depth = next_edges.size();
        auto best_it = best.find(endnode.value);
        if (best_it != best.end()) {
          const auto& best_depth = best_it->second.first;
          const auto& best_distance = best_it->second.second;
          if (next_depth >= best_depth && next_distance >= best_distance) {
            continue;
          }
        }
        best[endnode.value] = {next_depth, next_distance};

        if (next_depth < kMaxConnectorEdges) {
          queue.push_back({endnode, std::move(next_edges), next_distance});
        }
      }

      if (out.size() >= kMaxAlternateConnectors) {
        break;
      }
    }
  }

  if (!out.empty()) {
    std::sort(out.begin(), out.end(), [](const auto& a, const auto& b) {
      const int a_idx = a.entry_segment_index >= 0 ? a.entry_segment_index : INT32_MAX;
      const int b_idx = b.entry_segment_index >= 0 ? b.entry_segment_index : INT32_MAX;
      return a_idx < b_idx;
    });
    if (out.size() > kMaxAlternateConnectors) {
      out.resize(kMaxAlternateConnectors);
    }
  }
}

} // namespace

class TileGetterWrapper : public valhalla::baldr::tile_getter_t {
public:
  TileGetterWrapper(ValhallaMobileHttpClient* http_client, bool is_gzipped)
      : http_client(http_client), is_gzipped(is_gzipped) {}

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
  }

private:
  bool is_gzipped;
  ValhallaMobileHttpClient* http_client;
};


ValhallaActor::ValhallaActor(const std::string& config_path, ValhallaMobileHttpClient* http_client) {
  std::string config_file(config_path);

  boost::property_tree::ptree config;
  rapidjson::read_json(config_file, config);

  auto mjolnir_config = config.get_child("mjolnir");
  graph_reader = std::make_unique<valhalla::baldr::GraphReader>(
    mjolnir_config,
    std::make_unique<TileGetterWrapper>(http_client, mjolnir_config.get<bool>("tile_url_gz", false))
  );
  actor = std::make_unique<valhalla::tyr::actor_t>(config, *graph_reader, true);
}

std::string ValhallaActor::route(const std::string& request) {
  std::string req = std::string(request);

  valhalla::Api api;
  std::string result = actor->route(req, nullptr, &api);

  try {
    std::vector<std::vector<std::tuple<StructureType, double, double, bool>>> all_routes_structures;
    std::vector<std::vector<RouteConnector>> all_routes_connectors;
    const size_t num_routes = api.trip().routes_size();
    for (size_t i = 0; i < num_routes; ++i) {
      std::vector<std::tuple<StructureType, double, double, bool>> route_structs;
      extract_structures(api, *graph_reader, i, route_structs);
      all_routes_structures.push_back(std::move(route_structs));
    }

    if (num_routes > 1) {
      all_routes_connectors.resize(num_routes);

      std::unordered_set<uint64_t> main_edge_ids;
      std::unordered_set<uint64_t> main_node_ids;
      std::vector<valhalla::midgard::PointLL> main_route_shape;
      collect_route_graph(api, *graph_reader, 0, main_edge_ids, main_node_ids, &main_route_shape);

      for (size_t i = 1; i < num_routes; ++i) {
        std::unordered_set<uint64_t> alt_edge_ids;
        std::unordered_set<uint64_t> alt_node_ids;
        collect_route_graph(api, *graph_reader, i, alt_edge_ids, alt_node_ids, nullptr);

        std::vector<RouteConnector> connectors;
        collect_connectors(main_node_ids, main_edge_ids, alt_node_ids, alt_edge_ids,
                           *graph_reader, main_route_shape, connectors);
        all_routes_connectors[i] = std::move(connectors);
      }
    }

    add_structures_to_json(result, all_routes_structures, all_routes_connectors);
  } catch (const std::exception& e) {
    printf("[ValhallaActor] structures extraction std::exception: %s\n", e.what());
  } catch (...) {
    printf("[ValhallaActor] structures extraction unknown exception\n");
  }

  return result;
}