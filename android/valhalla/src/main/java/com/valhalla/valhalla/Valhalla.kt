package com.valhalla.valhalla

import android.content.Context
import com.osrm.api.models.RouteResponse as OsrmRouteResponse
import com.squareup.moshi.Moshi
import com.squareup.moshi.kotlin.reflect.KotlinJsonAdapterFactory
import com.valhalla.api.models.DirectionsOptions
import com.valhalla.api.models.RouteRequest
import com.valhalla.api.models.RouteResponse
import com.valhalla.config.models.ValhallaConfig
import com.valhalla.valhalla.config.ValhallaConfigManager

/**
 * Main entry point for the Valhalla routing engine on Android.
 *
 * This class provides a Kotlin interface to the native Valhalla C++ routing engine. It handles
 * configuration management, JSON serialization, and routing requests.
 *
 * @param context The Android context used for file system operations and configuration management.
 * @param config The Valhalla configuration specifying tile locations and routing options.
 * @param valhallaConfigManager Manages the Valhalla configuration file on the device. Defaults to a
 *   new instance.
 * @param moshi JSON serialization adapter. Defaults to a Moshi instance with Kotlin reflection
 *   support.
 * @see ValhallaConfig
 * @see ValhallaConfigManager
 * @see RouteRequest
 * @see ValhallaResponse
 */
class Valhalla(
    context: Context,
    config: ValhallaConfig,
    valhallaConfigManager: ValhallaConfigManager = ValhallaConfigManager(context),
    private val moshi: Moshi = Moshi.Builder().add(KotlinJsonAdapterFactory()).build()
) {

  private val valhallaActor: ValhallaActorProviding

  init {
    valhallaConfigManager.writeConfig(config)
    valhallaActor = ValhallaActor(valhallaConfigManager.getAbsolutePath())
  }

  /**
   * Fetch a route from Valhalla.
   *
   * This function returns a sealed class with the format you designated. Currently this only
   * supports [ValhallaResponse.Json] and [ValhallaResponse.Osrm] formats.
   *
   * @param request The Valhalla routing request containing locations, costing model, and options.
   * @return The route response wrapped in a [ValhallaResponse] sealed class based on the requested
   *   format.
   * @throws ValhallaException.Internal if the Valhalla engine returns an error response.
   * @throws ValhallaException.InvalidError if an error response cannot be parsed.
   * @throws ValhallaException.InvalidResponse if the response JSON cannot be parsed.
   * @throws ValhallaException.NotSupported if an unsupported format (GPX or PBF) is requested.
   * @see RouteRequest
   * @see ValhallaResponse
   * @see DirectionsOptions.Format
   */
  fun route(request: RouteRequest): ValhallaResponse {
    val encodedRequest = moshi.adapter(RouteRequest::class.java).toJson(request)
    val rawResponse = valhallaActor.route(encodedRequest)

    // Check for error response in Valhalla format.
    // OSRM has a code and message like the valhalla error, but it's not the same format.
    // If the response contains routes, it's a valid OSRM response.
    if (rawResponse.contains("code") and !rawResponse.contains("routes")) {
      val error = moshi.adapter(ErrorResponse::class.java).fromJson(rawResponse)
      error?.let { throw ValhallaException.Internal(it) }
      throw ValhallaException.InvalidError()
    }

    return when (request.format?.toString()?.lowercase()) {
      "gpx" -> throw ValhallaException.NotSupported()
      "osrm" -> {
        val osrmResponse =
            moshi.adapter(OsrmRouteResponse::class.java).fromJson(rawResponse)
                ?: throw ValhallaException.InvalidResponse()
        ValhallaResponse.Osrm(osrmResponse)
      }

      "pbf" -> throw ValhallaException.NotSupported()
      // else includes the default Valhalla JSON response.
      else -> {
        val valhallaResponse =
            moshi.adapter(RouteResponse::class.java).fromJson(rawResponse)
                ?: throw ValhallaException.InvalidResponse()
        ValhallaResponse.Json(valhallaResponse)
      }
    }
  }

  data class Structure(
      val type: String,
      val lat: Double,
      val lon: Double,
      val take: Boolean,
  )

  private data class TripEnvelope(
      val trip: Trip?,
      val structures: List<Structure>?,
  ) {
    data class Trip(
        val structures: List<Structure>?,
    )
  }

  /**
   * Routes and returns bridge/tunnel structures from the first alternative separately.
   *
  * The native layer attaches a `structures` array to each trip in the JSON response.
   * This returns the structures from the first (best) alternative.
   */
  fun routeWithStructures(request: RouteRequest): Pair<ValhallaResponse, List<Structure>> {
    val encodedRequest = moshi.adapter(RouteRequest::class.java).toJson(request)
    val rawResponse = valhallaActor.route(encodedRequest)

    // Check for error response in Valhalla format.
    if (rawResponse.contains("code") and !rawResponse.contains("routes")) {
      val error = moshi.adapter(ErrorResponse::class.java).fromJson(rawResponse)
      error?.let { throw ValhallaException.Internal(it) }
      throw ValhallaException.InvalidError()
    }

    // Extract structures from trip (primary route) or alternatives if available.
    var structures: List<Structure> = emptyList()
    try {
      val json = org.json.JSONObject(rawResponse)
      val rootEnvelope = moshi.adapter(TripEnvelope::class.java).fromJson(rawResponse)
      if (rootEnvelope != null) {
        val tripStructures = rootEnvelope.trip?.structures
        if (!tripStructures.isNullOrEmpty()) {
          structures = tripStructures
        } else {
          val rootStructures = rootEnvelope.structures
          if (!rootStructures.isNullOrEmpty()) {
            structures = rootStructures
          }
        }
      }

      if (structures.isEmpty()) {
        val altKeys = arrayOf("alternates", "alternatives")
        for (key in altKeys) {
          val alternatives = json.optJSONArray(key)
          if (alternatives != null && alternatives.length() > 0) {
            val firstAlt = alternatives.getJSONObject(0).toString()
            val altEnvelope = moshi.adapter(TripEnvelope::class.java).fromJson(firstAlt)
            val tripStructures = altEnvelope?.trip?.structures
            structures = if (!tripStructures.isNullOrEmpty()) {
              tripStructures
            } else {
              altEnvelope?.structures ?: emptyList()
            }
            break
          }
        }
      }
    } catch (e: Exception) {
      // Silently ignore JSON parsing errors, just return empty structures.
    }

    val routeResponse =
        when (request.format?.toString()?.lowercase()) {
          "gpx" -> throw ValhallaException.NotSupported()
          "osrm" -> {
            val osrmResponse =
                moshi.adapter(OsrmRouteResponse::class.java).fromJson(rawResponse)
                    ?: throw ValhallaException.InvalidResponse()
            ValhallaResponse.Osrm(osrmResponse)
          }

          "pbf" -> throw ValhallaException.NotSupported()
          else -> {
            val valhallaResponse =
                moshi.adapter(RouteResponse::class.java).fromJson(rawResponse)
                    ?: throw ValhallaException.InvalidResponse()
            ValhallaResponse.Json(valhallaResponse)
          }
        }

    return Pair(routeResponse, structures)
  }
}
