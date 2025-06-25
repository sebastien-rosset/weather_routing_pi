/***************************************************************************
 *   Copyright (C) 2015 by OpenCPN development team                        *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 3 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301,  USA.         *
 ***************************************************************************/

#include <wx/wx.h>
#include <wx/log.h>

#include <unordered_map>
#include <cmath>
#include <sstream>
#include <cstdint>
#include <chrono>
#include <vector>
#include <tuple>
#include <algorithm>

#include "ConstraintChecker.h"
#include "WeatherDataProvider.h"
#include "RouteMap.h"
#include "Utilities.h"

#include "georef.h"
#include "ocpn_plugin.h"

// Quantize to 1e-5 deg (about 1m)
constexpr double QUANT = 1e5;

/**
 * The distance field is a spatial optimization to speed up land/water boundary
 * checking during weather routing calculations. Instead of calling the
 * expensive PlugIn_GSHHS_CrossesLand() function for every route segment, we
 * maintain a cached grid-based approximation of coast proximity.
 *
 * Design rationale:
 * - Weather routing checks thousands of route segments for land crossings
 * - Each land check is computationally expensive (coastline vector
 *   calculations)
 * - Many route segments are either clearly in water or clearly on land
 * - Distance field allows fast elimination of obvious cases
 *
 * Grid-based approach:
 * - World is divided into grid cells of DISTANCE_FIELD_RESOLUTION (0.01° ≈ 1km)
 * - Each grid point stores coast proximity classification
 * - Uses LRU cache to manage memory usage
 *
 * Optimization strategy - three-tier performance:
 * 1. Segment cache: Previously computed exact results
 * 2. Distance field heuristics: Fast approximation eliminates 70-90% of exact
 * calculations
 * 3. Exact GSHHS computation: Accurate but expensive, used only when necessary
 */

/**
 * Defines the spatial granularity of the distance field cache grid.
 * Each grid cell represents approximately 1km × 1km at the equator.
 */
constexpr double DISTANCE_FIELD_RESOLUTION = 0.01;

/**
 * Controls memory usage of the distance field cache through LRU eviction.
 * Each cache entry stores distance-to-land estimates for one grid cell.
 */
constexpr int DISTANCE_FIELD_MAX_CACHE_SIZE = 10000;

bool use_df_cache = false;  // Global flag to enable/disable distance field cache

/*
 * GridPoint: Spatial indexing for distance field cache.
 *
 * Converts geographic coordinates (lat/lon) to integer grid coordinates
 * for efficient hashing and lookup in the distance field cache.
 *
 * Quantization:
 * - Lat/lon coordinates are quantized to DISTANCE_FIELD_RESOLUTION grid
 * - This creates discrete "buckets" that nearby points map to the same grid
 *   cell
 * - Reduces cache size while maintaining reasonable spatial accuracy
 */
struct GridPoint {
  int lat_grid;
  int lon_grid;

  GridPoint(double lat, double lon) {
    lat_grid = static_cast<int>(std::round(lat / DISTANCE_FIELD_RESOLUTION));
    lon_grid = static_cast<int>(std::round(lon / DISTANCE_FIELD_RESOLUTION));
  }

  bool operator==(const GridPoint& o) const {
    return lat_grid == o.lat_grid && lon_grid == o.lon_grid;
  }
};

/*
 * Hash function for GridPoint to enable use in std::unordered_map
 *
 * Combines lat_grid and lon_grid into a single 64-bit hash value
 * using bit shifting to avoid hash collisions.
 */
template <>
struct std::hash<GridPoint> {
  std::size_t operator()(const GridPoint& p) const {
    return std::hash<uint64_t>()((static_cast<uint64_t>(p.lat_grid) << 32) |
                                 static_cast<uint64_t>(p.lon_grid));
  }
};

/**
 * Coast proximity classification for distance field optimization.
 */
enum class CoastProximity {
  SAFE_WATER,  ///< Far from land (>2km), entire segment is navigable
  ON_LAND,     ///< Grid point is on land, segment likely invalid
  NEAR_COAST   ///< Near coastline, requires exact land-crossing computation
};

/**
 * Stores the simplified coast proximity classification for a grid point.
 */
struct DistanceFieldEntry {
  /**
   * Coast proximity classification for routing decisions.
   * @see CoastProximity for meaning of each value
   */
  CoastProximity proximity;

  /** Timestamp for LRU cache management */
  std::chrono::steady_clock::time_point last_used;

  DistanceFieldEntry()
      : proximity(CoastProximity::NEAR_COAST),
        last_used(std::chrono::steady_clock::now()) {}

  DistanceFieldEntry(CoastProximity prox)
      : proximity(prox), last_used(std::chrono::steady_clock::now()) {}
};

/*
 * Dual-level caching architecture:
 *
 * 1. Distance field cache (distance_field_cache):
 *    - Maps GridPoint -> DistanceFieldEntry
 *    - Stores approximate distance to land for grid cells
 *    - Used for fast elimination of obvious cases
 *    - LRU to ensure cache size is bounded
 *
 * 2. Segment cache (land_cache):
 *    - Maps SegmentKey -> SegmentCacheEntry
 *    - Stores exact results from expensive PlugIn_GSHHS_CrossesLand() calls
 *    - Used for precise land crossing determination
 *    - LRU to ensure cache size is bounded
 *
 * Cache interaction:
 * The distance field acts as a "prefilter" - if we can determine from distance
 * field data that a segment definitely doesn't cross land (far from shore) or
 * definitely does cross land (deep inland), we can skip the expensive GSHHS
 * calculation and cache the result directly in the segment cache.
 */
static std::unordered_map<GridPoint, DistanceFieldEntry> distance_field_cache;

/** Original segment cache with size limit. */
struct SegmentCacheEntry {
  bool crosses_land;
  std::chrono::steady_clock::time_point last_used;

  SegmentCacheEntry()
      : crosses_land(false), last_used(std::chrono::steady_clock::now()) {}

  SegmentCacheEntry(bool crosses)
      : crosses_land(crosses), last_used(std::chrono::steady_clock::now()) {}
};

constexpr size_t MAX_SEGMENT_CACHE_SIZE = 10000;

struct SegmentKey {
  uint64_t packed;
  SegmentKey(double lat1, double lon1, double lat2, double lon2) {
    int la1 = static_cast<int>(std::round(lat1 * QUANT));
    int lo1 = static_cast<int>(std::round(lon1 * QUANT));
    int la2 = static_cast<int>(std::round(lat2 * QUANT));
    int lo2 = static_cast<int>(std::round(lon2 * QUANT));
    // Always store with smaller endpoint first for symmetry
    if (la1 > la2 || (la1 == la2 && lo1 > lo2)) {
      std::swap(la1, la2);
      std::swap(lo1, lo2);
    }
    // Pack four 16-bit signed ints into a 64-bit value
    packed =
        (static_cast<uint64_t>(static_cast<uint32_t>(la1) & 0xFFFF) << 48) |
        (static_cast<uint64_t>(static_cast<uint32_t>(lo1) & 0xFFFF) << 32) |
        (static_cast<uint64_t>(static_cast<uint32_t>(la2) & 0xFFFF) << 16) |
        (static_cast<uint64_t>(static_cast<uint32_t>(lo2) & 0xFFFF));
  }
  bool operator==(const SegmentKey& o) const { return packed == o.packed; }
};

template <>
struct std::hash<SegmentKey> {
  std::size_t operator()(const SegmentKey& k) const {
    // Use std::hash for uint64_t
    return std::hash<uint64_t>()(k.packed);
  }
};

static std::unordered_map<SegmentKey, SegmentCacheEntry> land_cache;

static size_t segment_cache_hits = 0, segment_cache_misses = 0,
              segment_cache_queries = 0;
static size_t df_hits = 0, df_misses = 0, df_queries = 0,
              df_safe_water_optimizations = 0;
static size_t segment_evictions = 0, distance_field_evictions = 0;
constexpr size_t LOG_INTERVAL = 1000;

// LRU eviction for segment cache
void evict_oldest_segment_entries() {
  if (land_cache.size() <= MAX_SEGMENT_CACHE_SIZE) return;

  std::vector<std::pair<SegmentKey, std::chrono::steady_clock::time_point>>
      entries;
  entries.reserve(land_cache.size());

  for (const auto& pair : land_cache) {
    entries.emplace_back(pair.first, pair.second.last_used);
  }

  // Sort by last_used time (oldest first)
  std::sort(
      entries.begin(), entries.end(),
      [](const std::pair<SegmentKey, std::chrono::steady_clock::time_point>& a,
         const std::pair<SegmentKey, std::chrono::steady_clock::time_point>&
             b) { return a.second < b.second; });

  // Remove oldest 20% of entries
  size_t to_remove = land_cache.size() - MAX_SEGMENT_CACHE_SIZE * 0.8;
  for (size_t i = 0; i < to_remove && i < entries.size(); ++i) {
    land_cache.erase(entries[i].first);
  }

  segment_evictions += to_remove;
}

// LRU eviction for distance field cache
void evict_oldest_distance_field_entries() {
  if (distance_field_cache.size() <= DISTANCE_FIELD_MAX_CACHE_SIZE) return;

  std::vector<std::pair<GridPoint, std::chrono::steady_clock::time_point>>
      entries;
  entries.reserve(distance_field_cache.size());

  for (const auto& pair : distance_field_cache) {
    entries.emplace_back(pair.first, pair.second.last_used);
  }

  std::sort(
      entries.begin(), entries.end(),
      [](const std::pair<GridPoint, std::chrono::steady_clock::time_point>& a,
         const std::pair<GridPoint, std::chrono::steady_clock::time_point>& b) {
        return a.second < b.second;
      });

  size_t to_remove =
      distance_field_cache.size() - DISTANCE_FIELD_MAX_CACHE_SIZE * 0.8;
  for (size_t i = 0; i < to_remove && i < entries.size(); ++i) {
    distance_field_cache.erase(entries[i].first);
  }

  distance_field_evictions += to_remove;
}

/**
 * Determines coast proximity category.
 * Needed for routing optimization decisions.
 *
 * @param lat, lon Geographic coordinates to query
 * @param proximity Output: coast proximity classification
 * @return true if cache hit, false if computed new value
 */
bool query_distance_field(double lat, double lon, CoastProximity& proximity) {
  GridPoint gp(lat, lon);

  auto it = distance_field_cache.find(gp);
  if (it != distance_field_cache.end()) {
    it->second.last_used = std::chrono::steady_clock::now();
    proximity = it->second.proximity;
    ++df_hits;
    return true;
  }

  // Compute coast proximity for this grid point
  double grid_lat = gp.lat_grid * DISTANCE_FIELD_RESOLUTION;
  double grid_lon = gp.lon_grid * DISTANCE_FIELD_RESOLUTION;

  // Check if grid point is on land
  bool is_land =
      PlugIn_GSHHS_CrossesLand(grid_lat, grid_lon, grid_lat, grid_lon);

  CoastProximity computed_proximity;

  if (is_land) {
    computed_proximity = CoastProximity::ON_LAND;
  } else {
    /*
     * For water points, we only need to distinguish between:
     * - SAFE_WATER: Far enough from land that segments are clearly navigable
     * - NEAR_COAST: Close to land, requires exact land-crossing computation
     *
     * We test at 2.5 grid cells distance in 8 directions (~2.8km at equator).
     * If no land is found, the point is classified as SAFE_WATER. This ensures
     * consistency with the grid resolution and provides a conservative safety
     * margin.
     */
    const double SAFE_WATER_TEST_DISTANCE = 2.5 * DISTANCE_FIELD_RESOLUTION;
    bool found_nearby_land = false;
    int land_directions = 0;

    for (int angle = 0; angle < 360; angle += 45) {
      double rad = angle * M_PI / 180.0;
      double sample_lat = grid_lat + SAFE_WATER_TEST_DISTANCE * cos(rad);
      double sample_lon = grid_lon + SAFE_WATER_TEST_DISTANCE * sin(rad);

      bool hits_land =
          PlugIn_GSHHS_CrossesLand(grid_lat, grid_lon, sample_lat, sample_lon);
      if (hits_land) {
        found_nearby_land = true;
        land_directions++;
      }
    }

    computed_proximity = found_nearby_land ? CoastProximity::NEAR_COAST
                                           : CoastProximity::SAFE_WATER;
  }

  // Cache the result
  if (distance_field_cache.size() >= DISTANCE_FIELD_MAX_CACHE_SIZE) {
    evict_oldest_distance_field_entries();
  }

  distance_field_cache.emplace(gp, DistanceFieldEntry(computed_proximity));

  proximity = computed_proximity;
  ++df_misses;
  return false;  // Cache miss
}

void log_cache_stats() {
  double segment_hit_rate =
      segment_cache_queries ? (double)segment_cache_hits / segment_cache_queries
                            : 0.0;
  // How often segments using distance field avoided expensive GSHHS calls.
  double df_optimization_rate =
      df_queries ? (double)df_safe_water_optimizations / df_queries : 0.0;

  wxLogMessage(
      "WeatherRouting Segment cache: queries=%zu, hits=%zu, misses=%zu, "
      "size=%zu, hitrate=%.1f%%, evictions=%zu",
      segment_cache_queries, segment_cache_hits, segment_cache_misses,
      land_cache.size(), 100.0 * segment_hit_rate, segment_evictions);

  wxLogMessage(
      "WeatherRouting Distance field cache: queries=%zu, safe_water_hits=%zu, "
      "misses=%zu, size=%zu, safe_water_optimization_rate=%.1f%%, "
      "evictions=%zu",
      df_queries, df_safe_water_optimizations, df_misses,
      distance_field_cache.size(), 100.0 * df_optimization_rate,
      distance_field_evictions);
}

void maintain_land_cache() {
  evict_oldest_segment_entries();
  evict_oldest_distance_field_entries();

  static size_t last_log_queries = 0;
  if (segment_cache_queries - last_log_queries > LOG_INTERVAL) {
    log_cache_stats();
    last_log_queries = segment_cache_queries;
  }
}

/**
 * Split a long route segment into sub-segments to ensure reliable distance
 * field heuristics.
 *
 * The distance field optimization works best for short segments where the
 * midpoint distance is representative of the entire segment. For long segments,
 * we split them into chunks that are small relative to the distance field grid.
 *
 * @param lat1 Start latitude (degrees)
 * @param lon1 Start longitude (degrees)
 * @param lat2 End latitude (degrees)
 * @param lon2 End longitude (degrees)
 * @param subsegments Output vector of (lat1,lon1,lat2,lon2) tuples for each
 * sub-segment
 *
 * Long segments could have a midpoint far from land but still cross land
 * boundaries. By splitting long segments, we ensure that each sub-segment is ≤
 * grid resolution, making midpoint-based heuristics reliable
 */
void split_segment_for_distance_field(
    double lat1, double lon1, double lat2, double lon2,
    std::vector<std::tuple<double, double, double, double>>& subsegments) {
  // Maximum segment length for reliable distance field heuristics
  // Use same resolution as distance field grid to ensure consistency
  // This ensures midpoint-based heuristics are only applied to segments
  // that are small relative to the grid resolution
  const double SEGMENT_SPLIT_THRESHOLD = DISTANCE_FIELD_RESOLUTION;  // degrees

  double dlat = lat2 - lat1;
  double dlon = lon2 - lon1;
  double segment_length = std::sqrt(dlat * dlat + dlon * dlon);

  if (segment_length <= SEGMENT_SPLIT_THRESHOLD) {
    // Segment is short enough - use as-is
    subsegments.emplace_back(lat1, lon1, lat2, lon2);
    return;
  }

  // Split into equal parts, ensuring each part is <= threshold
  int num_parts =
      static_cast<int>(std::ceil(segment_length / SEGMENT_SPLIT_THRESHOLD));

  for (int i = 0; i < num_parts; ++i) {
    double t1 = static_cast<double>(i) / num_parts;
    double t2 = static_cast<double>(i + 1) / num_parts;

    double sub_lat1 = lat1 + t1 * dlat;
    double sub_lon1 = lon1 + t1 * dlon;
    double sub_lat2 = lat1 + t2 * dlat;
    double sub_lon2 = lon1 + t2 * dlon;

    subsegments.emplace_back(sub_lat1, sub_lon1, sub_lat2, sub_lon2);
  }
}

/**
 * Determine if a route segment is invalid due to land interference.
 *
 * @param lat1 Latitude of segment start point (degrees)
 * @param lon1 Longitude of segment start point (degrees)
 * @param lat2 Latitude of segment end point (degrees)
 * @param lon2 Longitude of segment end point (degrees)
 * @return true if segment crosses land or is entirely on land (invalid route),
 *         false if segment is entirely in navigable water (valid route)
 *
 * Optimization strategy:
 *
 * Segment cache check (o(1)):
 * - First check if we've computed this exact segment before
 * - If cache hit, return immediately with known result
 *
 * Segment splitting for long routes:
 * - Split long segments into sub-segments to ensure distance field reliability
 * - Each sub-segment is short enough that midpoint distance is representative
 * - Check each sub-segment
 *
 * Distance field optimization (o(1) if cached):
 * - Query distance field at segment midpoint (for short segments only)
 * - Apply safe water optimization for obvious cases:
 *
 *   a) Safe water optimization:
 *      If midpoint is >2km from land, entire segment is in safe water
 *      → Return false (valid route)
 *
 * Exact computation fallback:
 * - For near-coastline segments or on-land grid points, use expensive
 * PlugIn_GSHHS_CrossesLand()
 * - Cache the exact result for future use
 */
bool Cached_CrossesLand(double lat1, double lon1, double lat2, double lon2) {
  SegmentKey key(lat1, lon1, lat2, lon2);
  ++segment_cache_queries;

  // Check segment cache first.
  auto it = land_cache.find(key);
  if (it != land_cache.end()) {
    it->second.last_used = std::chrono::steady_clock::now();
    ++segment_cache_hits;
    return it->second.crosses_land;
  }

  if (use_df_cache) {
    // For long segments, split them into sub-segments to ensure distance field
    // reliability.
    std::vector<std::tuple<double, double, double, double>> subsegments;
    split_segment_for_distance_field(lat1, lon1, lat2, lon2, subsegments);

    if (subsegments.size() > 1) {
      // Long segment - check each sub-segment recursively
      bool any_crosses_land = false;
      for (size_t i = 0; i < subsegments.size(); ++i) {
        const auto& subseg = subsegments[i];
        if (Cached_CrossesLand(std::get<0>(subseg), std::get<1>(subseg),
                               std::get<2>(subseg), std::get<3>(subseg))) {
          any_crosses_land = true;
          break;
        }
      }

      // Cache the result for the original segment.
      if (land_cache.size() >= MAX_SEGMENT_CACHE_SIZE) {
        evict_oldest_segment_entries();
      }
      land_cache.emplace(key, SegmentCacheEntry(any_crosses_land));
      ++segment_cache_misses;
      return any_crosses_land;
    }

    // Short segment - use distance field optimization
    double mid_lat = (lat1 + lat2) / 2.0;
    double mid_lon = (lon1 + lon2) / 2.0;
    CoastProximity proximity;
    bool df_available = query_distance_field(mid_lat, mid_lon, proximity);
    ++df_queries;

    /*
     * We're checking if a route segment is INVALID due to land. A segment is
     * invalid if it either:
     * 1. Crosses a land boundary (water → land → water)
     * 2. Is entirely on land
     *
     * Safe water optimization:
     * If the midpoint is far from land (SAFE_WATER), the entire segment must be
     * in navigable water.
     */
    if (df_available && proximity == CoastProximity::SAFE_WATER) {
      // Midpoint is far from land, entire segment must be in safe water
      if (land_cache.size() >= MAX_SEGMENT_CACHE_SIZE) {
        evict_oldest_segment_entries();
      }
      land_cache.emplace(key, SegmentCacheEntry(false));
      ++segment_cache_misses;
      ++df_safe_water_optimizations;  // Track successful optimization
      return false;
    }
  }
  /*
   * Exact computation required for:
   * - Segments near coastline (proximity == NEAR_COAST)
   * - Segments where midpoint grid cell contains land (proximity == ON_LAND)
   * - Distance field data not available (cache miss + computation failed)
   *
   * For segments near coastlines or in mixed land/water grid cells, only exact
   * computation can reliably determine if the segment crosses land boundaries.
   * Grid-based approximations are insufficient due to quantization effects.
   *
   * Uses expensive but accurate PlugIn_GSHHS_CrossesLand() function and
   * caches the exact result for future use. This maintains correctness
   * while still providing optimization for clear safe-water cases.
   */
  ++segment_cache_misses;
  bool result = PlugIn_GSHHS_CrossesLand(lat1, lon1, lat2, lon2);

  // Cache the result
  if (land_cache.size() >= MAX_SEGMENT_CACHE_SIZE) {
    evict_oldest_segment_entries();
  }
  land_cache.emplace(key, SegmentCacheEntry(result));
  return result;
}

bool ConstraintChecker::CheckSwellConstraint(
    RouteMapConfiguration& configuration, double lat, double lon, double& swell,
    PropagationError& error_code) {
  swell = WeatherDataProvider::GetSwell(configuration, lat, lon);
  if (swell > configuration.MaxSwellMeters) {
    wxLogGeneric(
        wxLOG_Debug,
        "WeatherRouting: Swell constraint exceeded at lat=%.6f lon=%.6f: %.2fm "
        "> %.2fm max",
        lat, lon, swell, configuration.MaxSwellMeters);
    error_code = PROPAGATION_EXCEEDED_MAX_SWELL;
    return false;
  }
  return true;
}

bool ConstraintChecker::CheckMaxLatitudeConstraint(
    RouteMapConfiguration& configuration, double lat,
    PropagationError& error_code) {
  if (fabs(lat) > configuration.MaxLatitude) {
    wxLogGeneric(
        wxLOG_Debug,
        "WeatherRouting: Max latitude constraint exceeded: %.6f > %.6f",
        fabs(lat), configuration.MaxLatitude);
    error_code = PROPAGATION_EXCEEDED_MAX_LATITUDE;
    return false;
  }
  return true;
}

bool ConstraintChecker::CheckCycloneTrackConstraint(
    RouteMapConfiguration& configuration, double lat, double lon, double dlat,
    double dlon) {
  if (configuration.AvoidCycloneTracks &&
      RouteMap::ClimatologyCycloneTrackCrossings) {
    int crossings = RouteMap::ClimatologyCycloneTrackCrossings(
        lat, lon, dlat, dlon, configuration.time,
        configuration.CycloneMonths * 30 + configuration.CycloneDays);
    if (crossings > 0) {
      return false;
    }
  }
  return true;
}

bool ConstraintChecker::CheckMaxCourseAngleConstraint(
    RouteMapConfiguration& configuration, double dlat, double dlon) {
  if (configuration.MaxCourseAngle < 180) {
    double bearing;
    // this is faster than gc distance, and actually works better in higher
    // latitudes
    double d1 = dlat - configuration.StartLat,
           d2 = dlon - configuration.StartLon;
    d2 *= cos(deg2rad(dlat)) / 2;  // correct for latitude
    bearing = rad2deg(atan2(d2, d1));

    if (fabs(heading_resolve(configuration.StartEndBearing - bearing)) >
        configuration.MaxCourseAngle) {
      return false;
    }
  }
  return true;
}

bool ConstraintChecker::CheckMaxDivertedCourse(
    RouteMapConfiguration& configuration, double dlat, double dlon) {
  if (configuration.MaxDivertedCourse < 180) {
    double bearing, dist;
    double bearing1, dist1;

    double d1 = dlat - configuration.EndLat, d2 = dlon - configuration.EndLon;
    d2 *= cos(deg2rad(dlat)) / 2;  // correct for latitude
    bearing = rad2deg(atan2(d2, d1));
    dist = sqrt(pow(d1, 2) + pow(d2, 2));

    d1 = configuration.StartLat - dlat, d2 = configuration.StartLon - dlon;
    bearing1 = rad2deg(atan2(d2, d1));
    dist1 = sqrt(pow(d1, 2) + pow(d2, 2));

    double term = (dist1 + dist) / dist;
    term = pow(term / 16, 4) + 1;  // make 1 until the end, then make big

    if (fabs(heading_resolve(bearing1 - bearing)) >
        configuration.MaxDivertedCourse * term) {
      return false;
    }
  }
  return true;
}

bool ConstraintChecker::CheckLandConstraint(
    RouteMapConfiguration& configuration, double lat, double lon, double dlat1,
    double dlon1, double cog) {
  if (configuration.DetectLand) {
    double ndlon1 = dlon1;
    if (ndlon1 > 360) {
      ndlon1 -= 360;
    }
    if (Cached_CrossesLand(lat, lon, dlat1, ndlon1)) {
      return false;
    }
    double distSecure = configuration.SafetyMarginLand;
    double latBorderUp1, lonBorderUp1, latBorderUp2, lonBorderUp2;
    double latBorderDown1, lonBorderDown1, latBorderDown2, lonBorderDown2;
    ll_gc_ll(lat, lon, heading_resolve(cog) - 90, distSecure, &latBorderUp1,
             &lonBorderUp1);
    ll_gc_ll(dlat1, dlon1, heading_resolve(cog) - 90, distSecure, &latBorderUp2,
             &lonBorderUp2);
    ll_gc_ll(lat, lon, heading_resolve(cog) + 90, distSecure, &latBorderDown1,
             &lonBorderDown1);
    ll_gc_ll(dlat1, dlon1, heading_resolve(cog) + 90, distSecure,
             &latBorderDown2, &lonBorderDown2);
    if (Cached_CrossesLand(latBorderUp1, lonBorderUp1, latBorderUp2,
                           lonBorderUp2) ||
        Cached_CrossesLand(latBorderDown1, lonBorderDown1, latBorderDown2,
                           lonBorderDown2) ||
        Cached_CrossesLand(latBorderUp1, lonBorderUp1, latBorderDown2,
                           lonBorderDown2) ||
        Cached_CrossesLand(latBorderDown1, lonBorderDown1, latBorderUp2,
                           lonBorderUp2)) {
      return false;
    }
  }
  return true;
}

bool ConstraintChecker::CheckMaxTrueWindConstraint(
    RouteMapConfiguration& configuration, double twsOverWater,
    PropagationError& error_code) {
  if (twsOverWater > configuration.MaxTrueWindKnots) {
    error_code = PROPAGATION_EXCEEDED_MAX_WIND;
    return false;
  }
  return true;
}

bool ConstraintChecker::CheckMaxApparentWindConstraint(
    RouteMapConfiguration& configuration, double stw, double twa,
    double twsOverWater, PropagationError& error_code) {
  if (stw + twsOverWater > configuration.MaxApparentWindKnots &&
      Polar::VelocityApparentWind(stw, twa, twsOverWater) >
          configuration.MaxApparentWindKnots) {
    return false;
  }
  return true;
}

bool ConstraintChecker::CheckWindVsCurrentConstraint(
    RouteMapConfiguration& configuration, double twsOverWater,
    double twdOverWater, double currentSpeed, double currentDir,
    PropagationError& error_code) {
  if (configuration.WindVSCurrent) {
    /*
     * Calculate the wind vector (Wx, Wy) and ocean current vector (Cx, Cy).
     * these are already computed in GroundToWaterFrame could optimize by
     * reusing them.
     */
    double Wx = twsOverWater * cos(deg2rad(twdOverWater)),
           Wy = twsOverWater * sin(deg2rad(twdOverWater));
    double Cx = currentSpeed * cos(deg2rad(currentDir) + M_PI),
           Cy = currentSpeed * sin(deg2rad(currentDir) + M_PI);

    if (Wx * Cx + Wy * Cy + configuration.WindVSCurrent < 0) {
      error_code = PROPAGATION_EXCEEDED_WIND_VS_CURRENT;
      return false;
    }
  }
  return true;
}