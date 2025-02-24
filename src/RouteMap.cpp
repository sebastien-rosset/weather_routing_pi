/***************************************************************************
 *
 * Project:  OpenCPN Weather Routing plugin
 * Author:   Sean D'Epagnier
 *
 ***************************************************************************
 *   Copyright (C) 2016 by Sean D'Epagnier                                 *
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
 ***************************************************************************
 */

/* generate a datastructure which contains positions for
   isochron line segments which describe the position of the boat at a given
   time..

   Starting at a given location, propagate outwards in all directions.
   the outward propagation is guarenteed a closed region, and circular linked
   lists are used. If the route comes upon a boundary or reason to stop
   searching, then the point is flagged so that it is not propagated any
   further.

   To merge regions requires virtually the same algorithm for descrambling
   (normalizing) a single region.

   To normalize a region means that no two line segments intersect.

   For each segment go through and see if it intersects
   with any other line segment.  When it does the old route will follow
   the correct direction of the intersection on the intersected route,
   and the new region generated will be recursively normalized and then
   merged.

   A positive intersection comes in from the right.  Negative intersections
   signal negative regions.

   For each segment in a given route
   If the intersection occurs with the route and itself
   a new region is created with the same sign as the intersection
   and added to the list of either positive or negative subregions
   otherwise if the intersection occurs on different routes
   the intersecting route is merged into this one,
   swapping their connections

   Once we reach the end of the route, we can declare that it is complete,
   so in turn recursively normalize each inner subroute.  The subregions
   with the same sign are inner routes.  Once these regions are all normalized,
   the remaining regions with a different sign are the perminent subregions.
   Any inner routes remaining with matching sign can be discarded.

   Any outer subregions are also normalized to give outer regions
   with both signs which can be appended to the incomming lists

   Any remaining routes should be tested to ensure they are outside this one,
   Any inside routes may be discarded leaving only inverted subroutes
*/

#include <wx/wx.h>

#include <stdlib.h>
#include <math.h>
#include <map>

#include "Utilities.h"
#include "Boat.h"
#include "RouteMap.h"
#include "weather_routing_pi.h"

#include "georef.h"

#define distance(X, Y) sqrt((X) * (X) + (Y) * (Y))  // much faster than hypot

long RouteMapPosition::s_ID = 0;

extern Json::Value g_ReceivedJSONMsg;
extern wxString g_ReceivedMessage;

static Json::Value RequestGRIB(const wxDateTime& t, const wxString& what,
                               double lat, double lon) {
  Json::Value error;
  Json::Value v;
  Json::FastWriter writer;
  // brain dead wx is expecting time in local time
  wxDateTime time = t.FromUTC();
  if (!time.IsValid()) return error;

  v["Day"] = time.GetDay();
  v["Month"] = time.GetMonth();
  v["Year"] = time.GetYear();
  v["Hour"] = time.GetHour();
  v["Minute"] = time.GetMinute();
  v["Second"] = time.GetSecond();

  v["Source"] = "WEATHER_ROUTING_PI";
  v["Type"] = "Request";
  v["Msg"] = "GRIB_VALUES_REQUEST";
  v["lat"] = lat;
  v["lon"] = lon;
  v[what] = 1;

  SendPluginMessage("GRIB_VALUES_REQUEST", writer.write(v));
  if (g_ReceivedMessage != wxEmptyString &&
      g_ReceivedJSONMsg["Type"].asString() == "Reply") {
    return g_ReceivedJSONMsg;
  }
  return error;
}

// Return the swell height at the specified lat/long location.
// @return the swell height in meters. 0 if no data is available.
static double Swell(RouteMapConfiguration& configuration, double lat,
                    double lon) {
  WR_GribRecordSet* grib = configuration.grib;

  if (!grib && !configuration.RouteGUID.IsEmpty() && configuration.UseGrib) {
    Json::Value r = RequestGRIB(configuration.time, "SWELL", lat, lon);
    if (!r.isMember("SWELL")) return 0;
    return r["SWELL"].asDouble();
  }

  if (!grib) return 0;

  GribRecord* grh = grib->m_GribRecordPtrArray[Idx_HTSIGW];
  if (!grh) return 0;

  double height = grh->getInterpolatedValue(lon, lat, true);
  if (height == GRIB_NOTDEF) return 0;
  // yep swell data can be negative!
  if (height < 0.) return 0.;
  return height;
}

// Return the wind gust speed for the specified lat/long location, in knots.
static double Gust(RouteMapConfiguration& configuration, double lat,
                   double lon) {
  WR_GribRecordSet* grib = configuration.grib;
  double gust;

  if (!grib && !configuration.RouteGUID.IsEmpty() && configuration.UseGrib) {
    Json::Value r = RequestGRIB(configuration.time, "GUST", lat, lon);
    if (!r.isMember("GUST")) return NAN;
    gust = r["GUST"].asDouble();
  } else if (!grib)
    return NAN;
  else {
    GribRecord* grh = grib->m_GribRecordPtrArray[Idx_WIND_GUST];
    if (!grh) return NAN;

    gust = grh->getInterpolatedValue(lon, lat, true);
  }
  if (gust == GRIB_NOTDEF) return NAN;
  gust *= 3.6 / 1.852;  // knots
  return gust;
}

/**
 * Retrieves wind data from GRIB file at a specified location.
 *
 * Fetches wind direction and speed at the given latitude and longitude from a
 * GRIB file.
 *
 * @param configuration Route map configuration containing GRIB data and
 * settings
 * @param lat Latitude in degrees
 * @param lon Longitude in degrees
 * @param twd [out] Wind direction over ground in degrees (meteorological
 * convention)
 * @param tws [out] Wind speed over ground in knots
 *
 * @return true if wind data was successfully retrieved, false otherwise
 */
static bool GribWind(RouteMapConfiguration& configuration, double lat,
                     double lon, double& twd, double& tws) {
  WR_GribRecordSet* grib = configuration.grib;

  if (!grib && !configuration.RouteGUID.IsEmpty() && configuration.UseGrib) {
    Json::Value r = RequestGRIB(configuration.time, "WIND SPEED", lat, lon);
    if (!r.isMember("WIND SPEED")) return false;
    tws = r["WIND SPEED"].asDouble();

    if (!r.isMember("WIND DIR")) return false;
    twd = r["WIND DIR"].asDouble();
  } else if (!grib)
    return false;

  else if (!GribRecord::getInterpolatedValues(
               tws, twd, grib->m_GribRecordPtrArray[Idx_WIND_VX],
               grib->m_GribRecordPtrArray[Idx_WIND_VY], lon, lat))
    return false;

  tws *= 3.6 / 1.852;  // knots
#if 0
    // test
    tws = 0.;
    twd = 0.;
#endif
  return true;
}

enum { WIND, CURRENT };

static bool GribCurrent(RouteMapConfiguration& configuration, double lat,
                        double lon, double& currentDir, double& currentSpeed) {
  WR_GribRecordSet* grib = configuration.grib;

  if (!grib && !configuration.RouteGUID.IsEmpty() && configuration.UseGrib) {
    Json::Value r = RequestGRIB(configuration.time, "CURRENT SPEED", lat, lon);
    if (!r.isMember("CURRENT SPEED")) return false;
    currentSpeed = r["CURRENT SPEED"].asDouble();

    if (!r.isMember("CURRENT DIR")) return false;
    currentDir = r["CURRENT DIR"].asDouble();
  } else if (!grib)
    return false;

  else if (!GribRecord::getInterpolatedValues(
               currentSpeed, currentDir,
               grib->m_GribRecordPtrArray[Idx_SEACURRENT_VX],
               grib->m_GribRecordPtrArray[Idx_SEACURRENT_VY], lon, lat))
    return false;

  currentSpeed *= 3.6 / 1.852;  // knots
  currentDir += 180;
  if (currentDir > 360) currentDir -= 360;
  return true;
}

static inline bool Current(RouteMapConfiguration& configuration, double lat,
                           double lon, double& currentDir, double& currentSpeed,
                           int& data_mask) {
  if (!configuration.grib_is_data_deficient &&
      GribCurrent(configuration, lat, lon, currentDir, currentSpeed)) {
    data_mask |= Position::GRIB_CURRENT;
    return true;
  }

  if (configuration.ClimatologyType != RouteMapConfiguration::DISABLED &&
      RouteMap::ClimatologyData &&
      RouteMap::ClimatologyData(CURRENT, configuration.time, lat, lon,
                                currentDir, currentSpeed)) {
    data_mask |= Position::CLIMATOLOGY_CURRENT;
    return true;
  }

#if 0  // for now disable deficient current data as it's usefulness is not known
    // use deficient grib current if climatology is not available
    // unlike wind, we don't use current data from a different location
    // so only current data from a different time is allowed
    if(configuration.AllowDataDeficient &&
       configuration.grib_is_data_deficient && GribCurrent(configuration, lat, lon, currentDir, currentSpeed)) {
        data_mask |= Position::GRIB_CURRENT | Position::DATA_DEFICIENT_CURRENT;
        return true;
    }
#endif

  return false;
}

/**
 * Converts wind parameters from ground-relative to water-relative.
 *
 * This function transforms true wind (measured relative to ground) into
 * water-relative wind (relative to the moving water) by accounting for the
 * effect of sea currents. This is NOT the same as apparent wind experienced by
 * a moving vessel, which would also account for the vessel's own motion.
 * Sometimes localized currents can be strong enough to create a breeze which
 * can be sailed off even if there is no wind. The wind data is calculated from
 * the ground not the sea, it is then converted to speed over water which the
 * boat can feel. can be sailed even in the absence of true wind. Wind and
 * current direction are from x ie 180 is wind/current from the south
 *
 * @param twd True Wind Direction over ground (degrees, meteorological
 * convention: FROM direction)
 * @param tws True Wind Speed over ground (knots)
 * @param currentDir Current Direction (degrees, meteorological convention: FROM
 * direction)
 * @param currentSpeed Current Speed (knots) - Note: for correct calculation,
 * this should be negative of actual current speed
 * @param waterWindDir [out] Wind Direction relative to water (degrees,
 * meteorological convention)
 * @param waterWindSpeed [out] Wind Speed relative to water (knots)
 */
static void OverWater(double twd, double tws, double currentDir,
                      double currentSpeed, double& waterWindDir,
                      double& waterWindSpeed) {
  if (currentSpeed == 0) {  // short-cut if no currents
    waterWindDir = twd, waterWindSpeed = tws;
    return;
  }

  double Cx = currentSpeed * cos(deg2rad(currentDir)),
         Cy = currentSpeed * sin(deg2rad(currentDir));
  double Wx = tws * cos(deg2rad(twd)) - Cx, Wy = tws * sin(deg2rad(twd)) - Cy;
  waterWindDir = rad2deg(atan2(Wy, Wx));
  waterWindSpeed = distance(Wx, Wy);
}

/* provisions to compute boat movement over ground

   cog  - boat direction over ground
   BGV - boat speed over ground (gps velocity)  */
static void OverGround(double ctw, double stw, double currentDir,
                       double currentSpeed, double& cog, double& sog) {
  if (currentSpeed == 0) {  // short-cut if no currents
    cog = ctw, sog = stw;
    return;
  }

  double Cx = currentSpeed * cos(deg2rad(currentDir)),
         Cy = currentSpeed * sin(deg2rad(currentDir));
  double BGx = stw * cos(deg2rad(ctw)) + Cx, BGy = stw * sin(deg2rad(ctw)) + Cy;
  cog = rad2deg(atan2(BGy, BGx));
  sog = distance(BGx, BGy);
}

/* find intersection of two line segments
   if no intersection return 0, otherwise, 1 if the
   second line crosses from right to left, or -1 for left to right

   In the case that it is too close to determine, we find which endpoint
   is the problematic point (and will be deleted from the graph)
   -2: first point first seg
   -3: second point first seg
   2: first point second seg
   3: second point second seg

   Truth equations to calculate intersection (x, y)
   (y-y1) * (x2-x1) = (y2-y1) * (x-x1)
   (y-y3) * (x4-x3) = (y4-y3) * (x-x3)
*/
static inline int TestIntersectionXY(double x1, double y1, double x2, double y2,
                                     double x3, double y3, double x4,
                                     double y4) {
  double ax = x2 - x1, ay = y2 - y1;
  double bx = x3 - x4, by = y3 - y4;
  double cx = x1 - x3, cy = y1 - y3;

  double denom = ay * bx - ax * by;

#undef EPS
#undef EPS2
#define EPS 2e-16
#define EPS2 2e-8          // should be half the exponent of EPS
  if (fabs(denom) < EPS) { /* parallel or really close to parallel */
#define EPS3 1e-5
    if (fabs(ax) < EPS3 &&
        fabs(ay) < EPS3) /* first segment is a zero segment */
      return -2;

    if (fabs(bx) < EPS3 &&
        fabs(by) < EPS3) /* second segment is a zero segment */
      return 2;

    /* we already know from initial test we are overlapping,
       for parallel line segments, there is no way to tell
       which direction the intersection occurs */
#define PEPS 2e-14
    if (fabs((y1 * ax - ay * x1) * bx - (y3 * bx - by * x3) * ax) > PEPS)
      return 0; /* different intercepts, no intersection */

    /* can invalidate a point on either segment for overlapping parallel,
       we will always choose second segment */
    double dx = x2 - x3, dy = y2 - y3;
    double da = ax * ax + bx * bx, db = cx * cx + cy * cy,
           dc = dx * dx + dy * dy;
    if (db <= da && dc <= da) /* point 3 is between 1 and 2 */
      return 2;
    return 3;
  }

  double recip = 1 / denom;
  double na = (by * cx - bx * cy) * recip;
  if (na < -EPS2 || na > 1 + EPS2) return 0;

  double nb = (ax * cy - ay * cx) * recip;
  if (nb < -EPS2 || nb > 1 + EPS2) return 0;

  /* too close to call.. floating point loses bits with arithmetic so
     in this case we must avoid potential false guesses */
  if (na < EPS2) return -2;
  if (na > 1 - EPS2) return -3;
  if (nb < EPS2) return 2;
  if (nb > 1 - EPS2) return 3;

  return denom < 0 ? -1 : 1;
}

#define EPSILON (2e-11)

Position::Position(double latitude, double longitude, Position* p,
                   double pheading, double pbearing, int sp, int t, int dm,
                   bool df)
    : RoutePoint(latitude, longitude, sp, t, df),
      parent_heading(pheading),
      parent_bearing(pbearing),
      parent(p),
      propagated(false),
      copied(false),
      data_mask(dm) {
  lat -= fmod(lat, EPSILON);
  lon -= fmod(lon, EPSILON);
}

Position::Position(Position* p)
    : RoutePoint(p->lat, p->lon, p->polar, p->tacks, p->grib_is_data_deficient),
      parent_heading(p->parent_heading),
      parent_bearing(p->parent_bearing),
      parent(p->parent),
      propagated(p->propagated),
      copied(true),
      data_mask(p->data_mask) {}

/* sufficient for routemap uses only.. is this faster than below? if not, remove
 * it */
static inline int ComputeQuadrantFast(Position* p, Position* q) {
  int quadrant;
  if (q->lat < p->lat)
    quadrant = 0;
  else
    quadrant = 2;

  if (p->lon < q->lon) quadrant++;

  return quadrant;
}

#if 0
/* works for all ranges */
static int ComputeQuadrant(Position *p, Position *q)
{
    int quadrant;
    if(q->lat < p->lat)
        quadrant = 0;
    else
        quadrant = 2;

    double diff = p->lon - q->lon;
    while(diff < -180) diff += 360;
    while(diff >= 180) diff -= 360;
    
    if(diff < 0)
        quadrant++;

    return quadrant;
}
#endif

SkipPosition* Position::BuildSkipList() {
  /* build skip list of positions, skipping over strings of positions in
     the same quadrant */
  SkipPosition* skippoints = NULL;
  Position* p = this;
  int firstquadrant, lastquadrant = -1, quadrant;
  do {
    Position* q = p->next;
    quadrant = ComputeQuadrantFast(p, q);

    if (lastquadrant == -1)
      firstquadrant = lastquadrant = quadrant;
    else if (quadrant != lastquadrant) {
      SkipPosition* rs = new SkipPosition(p, quadrant);
      if (skippoints) {
        rs->prev = skippoints->prev;
        rs->next = skippoints;
        skippoints->prev->next = rs;
        skippoints->prev = rs;
      } else {
        skippoints = rs;
        rs->prev = rs->next = rs;
      }
      lastquadrant = quadrant;
    }
    p = q;
  } while (p != this);

  if (!skippoints) {
    SkipPosition* rs = new SkipPosition(p, quadrant);
    rs->prev = rs->next = rs;
    skippoints = rs;
  } else if (quadrant != firstquadrant) {
    SkipPosition* rs = new SkipPosition(p, firstquadrant);

    rs->prev = skippoints->prev;
    rs->next = skippoints;
    skippoints->prev->next = rs;
    skippoints->prev = rs;

    skippoints = rs;
  }
  return skippoints;
}

/**
 * Represents a wind rose summary of climatological wind data for a location.
 *
 * This data structure holds statistical wind data typically derived from
 * historical weather patterns. The data is organized into 8 directional sectors
 * (at 45° intervals), storing both wind speeds and frequency of occurrence.
 */
struct climatology_wind_atlas {
  double W[8];  //!< Probability/weight of wind occurring in each direction
                //!< sector (0-1)
  double
      VW[8];  //!< Most common wind speed (in knots) for each direction sector
  double storm;  //!< Probability of storm conditions (0-1)
  double calm;   //!< Probability of calm conditions (0-1)
  /**Central wind direction (in degrees) for each sector:
   * typically [0, 45, 90, 135, 180, 225, 270, 315]
   */
  double directions[8];
};

/**
 * Retrieves wind and current data for a specific location
 *
 * This function attempts to obtain both wind and current information at the
 * specified location using multiple data sources in order of preference:
 * 1. GRIB files (if available and not data deficient)
 * 2. Climatology average data
 * 3. Climatology wind atlas data
 * 4. Data deficient GRIB (if allowed)
 * 5. Parent position data (fallback)
 *
 * The function calculates both ground-relative (true) and water-relative
 * (apparent) wind values by accounting for current effects.
 *
 * @param configuration Route map configuration containing GRIB data,
 * climatology settings, etc.
 * @param p Pointer to the route point for which to retrieve data
 * @param twd [out] True Wind Direction over ground (degrees)
 * @param tws [out] True Wind Speed over ground (knots)
 * @param awd [out] Apparent Wind Direction through water (degrees)
 * @param aws [out] Apparent Wind Speed through water (knots)
 * @param cur [out] Current Direction (degrees)
 * @param spd [out] Current Speed (knots)
 * @param atlas [out] Climatology wind atlas data (filled if climatology data is
 * used)
 * @param data_mask [in/out] Bit flags indicating what data sources were used
 *
 * @return true if wind and current data were successfully retrieved, false
 * otherwise
 */
static inline bool ReadWindAndCurrents(
    RouteMapConfiguration& configuration, RoutePoint* p,
    /* normal data */
    double& twd, double& tws, double& W, double& VW, double& currentDir,
    double& currentSpeed, climatology_wind_atlas& atlas, int& data_mask) {
  /* read current data */
  if (!configuration.Currents || !Current(configuration, p->lat, p->lon,
                                          currentDir, currentSpeed, data_mask))
    currentDir = currentSpeed = 0;

  for (;;) {
    if (!configuration.grib_is_data_deficient &&
        GribWind(configuration, p->lat, p->lon, twd, tws)) {
      data_mask |= Position::GRIB_WIND;
      break;
    }

    if (configuration.ClimatologyType == RouteMapConfiguration::AVERAGE &&
        RouteMap::ClimatologyData &&
        RouteMap::ClimatologyData(WIND, configuration.time, p->lat, p->lon, twd,
                                  tws)) {
      twd = heading_resolve(twd);
      data_mask |= Position::CLIMATOLOGY_WIND;
      break;
    } else if (configuration.ClimatologyType >
                   RouteMapConfiguration::CURRENTS_ONLY &&
               RouteMap::ClimatologyWindAtlasData) {
      int windatlas_count = 8;
      double speeds[8];
      if (RouteMap::ClimatologyWindAtlasData(configuration.time, p->lat, p->lon,
                                             windatlas_count, atlas.directions,
                                             speeds, atlas.storm, atlas.calm)) {
        /* compute wind speeds over water with the given current */
        for (int i = 0; i < windatlas_count; i++) {
          double twd = i * 360 / windatlas_count;
          double tws = speeds[i] * configuration.WindStrength;
          OverWater(twd, tws, currentDir, -currentSpeed, atlas.W[i],
                    atlas.VW[i]);
        }

        /* find most likely wind direction */
        double max_direction = 0;
        int maxi = 0;
        for (int i = 0; i < windatlas_count; i++)
          if (atlas.directions[i] > max_direction) {
            max_direction = atlas.directions[i];
            maxi = i;
          }

        /* now compute next most likely wind octant (adjacent to most likely)
           and linearly interpolate speed and direction from these two octants,
           we use this as the most likely wind, and base wind direction for the
           map */
        int maxia = maxi + 1, maxib = maxi - 1;
        if (maxia == windatlas_count) maxia = 0;
        if (maxib < 0) maxib = windatlas_count - 1;

        if (atlas.directions[maxia] < atlas.directions[maxib]) maxia = maxib;

        double maxid =
            1 / (atlas.directions[maxi] / atlas.directions[maxia] + 1);
        double angle1 = atlas.W[maxia], angle2 = atlas.W[maxi];
        while (angle1 - angle2 > 180) angle1 -= 360;
        while (angle2 - angle1 > 180) angle2 -= 360;
        W = heading_resolve(maxid * angle1 + (1 - maxid) * angle2);
        VW = maxid * atlas.VW[maxia] + (1 - maxid) * atlas.VW[maxi];

        OverGround(W, VW, currentDir, currentSpeed, twd, tws);
        data_mask |= Position::CLIMATOLOGY_WIND;
        return true;
      }
    }

    if (!configuration.AllowDataDeficient) return false;

    /* try deficient grib if climatology failed */
    if (configuration.grib_is_data_deficient &&
        GribWind(configuration, p->lat, p->lon, twd, tws)) {
      data_mask |= Position::GRIB_WIND | Position::DATA_DEFICIENT_WIND;
      break;
    }
    Position* n = dynamic_cast<Position*>(p);
    if (!n || !n->parent) return false;
    p = n->parent;
  }
  tws *= configuration.WindStrength;

  OverWater(twd, tws, currentDir, -currentSpeed, W, VW);
  return true;
}

/* get data from a position for plotting */
bool RoutePoint::GetPlotData(RoutePoint* next, double dt,
                             RouteMapConfiguration& configuration,
                             PlotData& data) {
  data.lat = lat;
  data.lon = lon;
  data.tacks = tacks;
  data.polar = polar;

  data.WVHT = Swell(configuration, lat, lon);
  data.VW_GUST = Gust(configuration, lat, lon);
  data.delta = dt;

  climatology_wind_atlas atlas;
  int data_mask = 0;  // not used for plotting yet
  bool old = configuration.grib_is_data_deficient;
  configuration.grib_is_data_deficient = grib_is_data_deficient;
  if (!ReadWindAndCurrents(configuration, this, data.twd, data.tws, data.W,
                           data.VW, data.currentDir, data.currentSpeed, atlas,
                           data_mask)) {
    // I don't think this can ever be hit, because the data should have been
    // there for the position be be created in the first place
    printf("Wind/Current data failed for position!!!\n");
    configuration.grib_is_data_deficient = old;
    return false;
  }

  ll_gc_ll_reverse(lat, lon, next->lat, next->lon, &data.cog, &data.sog);
  if (dt == 0)
    data.sog = 0;
  else
    data.sog *= 3600 / dt;

  OverWater(data.cog, data.sog, data.currentDir, data.currentSpeed, data.ctw,
            data.stw);
  configuration.grib_is_data_deficient = old;
  return true;
}

bool RoutePoint::GetWindData(RouteMapConfiguration& configuration, double& W,
                             double& VW, int& data_mask) {
  double twd, tws, currentDir, currentSpeed;
  climatology_wind_atlas atlas;
  return ReadWindAndCurrents(configuration, this, twd, tws, W, VW, currentDir,
                             currentSpeed, atlas, data_mask);
}

bool RoutePoint::GetCurrentData(RouteMapConfiguration& configuration,
                                double& currentDir, double& currentSpeed,
                                int& data_mask) {
  double twd, tws, W, VW;
  climatology_wind_atlas atlas;
  return ReadWindAndCurrents(configuration, this, twd, tws, W, VW, currentDir,
                             currentSpeed, atlas, data_mask);
}

/**
 * Computes the boat speed and bearing based on wind, current, and polar data.
 *
 * This function calculates the boat's speed and bearing through water and over
 * ground using either direct polar lookups or climatology wind atlas data. It
 * handles tacking adjustments when the wind angle is too close to the bow.
 *
 * @param configuration Boat and route configuration settings
 * @param timeseconds Duration in seconds for the calculation
 * @param twd Wind direction over ground (degrees)
 * @param tws Wind speed over ground (knots)
 * @param W Wind direction through water (degrees)
 * @param VW Wind speed through water (knots)
 * @param currentDir Current direction (degrees)
 * @param currentSpeed Current speed (knots)
 * @param hdg Boat heading (degrees)
 * @param atlas Wind climatology atlas containing probability distributions
 * @param data_mask Bit mask indicating data sources (GRIB, climatology, etc.)
 * @param ctw Boat's bearing relative to true wind (W+hdg), initialized by
 * caller
 * @param stw Boat speed through water [out]
 * @param cog Boat bearing over ground [out]
 * @param sog Boat speed over ground [out]
 * @param dist Distance traveled over ground (nm) [out]
 * @param newpolar Index of the polar to use from the boat's polar array
 *
 * @return true if computation successful, false if NaN values detected
 */
static inline bool ComputeBoatSpeed(
    RouteMapConfiguration& configuration, double timeseconds, double twd,
    double tws, double W, double VW, double currentDir, double currentSpeed,
    double& hdg, climatology_wind_atlas& atlas, int data_mask, double& ctw,
    double& stw, double& cog, double& sog, double& dist, int newpolar) {
  Polar& polar = configuration.boat.Polars[newpolar];
  if ((data_mask & Position::CLIMATOLOGY_WIND) &&
      (configuration.ClimatologyType == RouteMapConfiguration::CUMULATIVE_MAP ||
       configuration.ClimatologyType ==
           RouteMapConfiguration::CUMULATIVE_MINUS_CALMS)) {
    /* build map */
    stw = 0;
    int windatlas_count = 8;
    for (int i = 0; i < windatlas_count; i++) {
      // Calculate relative wind angle (difference between heading and wind
      // direction).
      double dir = hdg - W + atlas.W[i];
      if (dir > 180) dir = 360 - dir;
      double VBc, mind = polar.MinDegreeStep();
      // if tacking
      if (fabs(dir) < mind)
        VBc = polar.Speed(mind, atlas.VW[i], true,
                          configuration.OptimizeTacking) *
              cos(deg2rad(mind)) / cos(deg2rad(dir));
      else
        VBc =
            polar.Speed(dir, atlas.VW[i], true, configuration.OptimizeTacking);
      // Accumulate weighted boat speed based on probability of each wind
      // direction
      stw += atlas.directions[i] * VBc;
    }

    if (configuration.ClimatologyType ==
        RouteMapConfiguration::CUMULATIVE_MINUS_CALMS)
      stw *= 1 - atlas.calm;
  } else {
    // Direct polar lookup - get boat speed from polar data for current heading
    // and wind speed.
    stw = polar.Speed(hdg, VW, true, configuration.OptimizeTacking);
  }

  /* failed to determine speed.. */
  if (std::isnan(ctw) || std::isnan(stw)) {
    // when does this hit??
    wxLogMessage("polar failed bad! W=%f VW=%f ctw=%f stw=%f", W, VW, ctw, stw);
    configuration.polar_failed = true;
    return false;  // ctw = stw = 0;
  }

  // Calculate boat movement over ground by combining boat speed with current.
  OverGround(ctw, stw, currentDir, currentSpeed, cog, sog);

  // Calculate distance traveled over ground based on speed and time.
  dist = sog * timeseconds / 3600.0;
  return true;
}

/**
 * Performs a single Runge-Kutta integration step for route propagation.
 *
 * Takes a position and calculates the next position after a time step using
 * current winds and boat polars. This function is used as part of the
 * 4th order Runge-Kutta integration.
 *
 * @param p Current position
 * @param timeseconds Time step in seconds
 * @param cog Course over ground (degrees)
 * @param dist Distance to travel (nm)
 * @param H Boat heading (degrees)
 * @param configuration Route configuration parameters
 * @param grib GRIB weather data
 * @param time Current time
 * @param newpolar Index of polar to use
 * @param rk_BG [out] New bearing over ground (degrees)
 * @param rk_dist [out] New distance traveled (nm)
 * @param data_mask [out] Mask indicating data sources used
 *
 * @return true if step was successful, false if step failed
 */
bool rk_step(Position* p, double timeseconds, double cog, double dist, double H,
             RouteMapConfiguration& configuration, WR_GribRecordSet* grib,
             const wxDateTime& time, int newpolar, double& rk_BG,
             double& rk_dist, int& data_mask) {
  double k1_lat, k1_lon;
  ll_gc_ll(p->lat, p->lon, cog, dist, &k1_lat, &k1_lon);

  double twd, tws, W, VW, currentDir, currentSpeed;
  climatology_wind_atlas atlas;
  Position rk(k1_lat, k1_lon,
              p->parent);  // parent so deficient data can find parent
  if (!ReadWindAndCurrents(configuration, &rk, twd, tws, W, VW, currentDir,
                           currentSpeed, atlas, data_mask))
    return false;

  double ctw = W + H; /* rotated relative to true wind */

  double stw, sog;  // outputs
  if (!ComputeBoatSpeed(configuration, timeseconds, twd, tws, W, VW, currentDir,
                        currentSpeed, H, atlas, data_mask, ctw, stw, rk_BG, sog,
                        rk_dist, newpolar))
    return false;

  return true;
}

/* propagate to the end position in the configuration, and return the number of
 * seconds it takes */
double Position::PropagateToEnd(RouteMapConfiguration& cf, double& H,
                                int& data_mask) {
  return PropagateToPoint(cf.EndLat, cf.EndLon, cf, H, data_mask, true);
}

static void DeletePoints(Position* point) {
  Position* p = point;
  do {
    Position* dp = p;
    p = p->next;
    delete dp;
  } while (p != point);
}

bool Position::Propagate(IsoRouteList& routelist,
                         RouteMapConfiguration& configuration) {
  /* already propagated from this position, don't need to again */
  if (propagated) return false;

  propagated = true;

  Position* points = NULL;
  /* through all angles relative to wind */
  int count = 0;

  double S = Swell(configuration, lat, lon);
  if (S > configuration.MaxSwellMeters) return false;

  if (fabs(lat) > configuration.MaxLatitude) return false;

  double twd, tws, W, VW, currentDir, currentSpeed;
  climatology_wind_atlas atlas;
  int data_mask = 0;
  if (!ReadWindAndCurrents(configuration, this, twd, tws, W, VW, currentDir,
                           currentSpeed, atlas, data_mask)) {
    configuration.wind_data_failed = true;
    return false;
  }

  if (VW > configuration.MaxTrueWindKnots) return false;

  if (configuration.WindVSCurrent) {
    /* Calculate the wind vector (Wx, Wy) and ocean current vector (Cx, Cy). */
    /* these are already computed in OverWater.. could optimize by reusing them
     */
    double Wx = VW * cos(deg2rad(W)), Wy = VW * sin(deg2rad(W));
    double Cx = currentSpeed * cos(deg2rad(currentDir) + M_PI),
           Cy = currentSpeed * sin(deg2rad(currentDir) + M_PI);

    if (Wx * Cx + Wy * Cy + configuration.WindVSCurrent < 0) return false;
  }

  bool first_avoid = true;
  Position* rp;

  double bearing1 = NAN, bearing2 = NAN;
  if (parent && configuration.MaxSearchAngle < 180) {
    bearing1 = heading_resolve(parent_bearing - configuration.MaxSearchAngle);
    bearing2 = heading_resolve(parent_bearing + configuration.MaxSearchAngle);
  }

  for (auto it = configuration.DegreeSteps.begin();
       it != configuration.DegreeSteps.end(); it++) {
    double timeseconds = configuration.UsedDeltaTime;
    double dist;

    double H = heading_resolve(*it);
    double ctw, stw, cog, sog;

    ctw = W + H; /* rotated relative to true wind */

    /* test to avoid extra computations related to backtracking */
    if (!std::isnan(bearing1)) {
      double bearing3 = heading_resolve(ctw);
      if ((bearing1 > bearing2 && bearing3 > bearing2 && bearing3 < bearing1) ||
          (bearing1 < bearing2 &&
           (bearing3 > bearing2 || bearing3 < bearing1))) {
        if (first_avoid) {
          /* add a position behind the lines to ensure our route intersects
             with the previous one to nicely merge the resulting graph */
          first_avoid = false;
          rp = new Position(this);
          double dp = .95;
          rp->lat = (1 - dp) * lat + dp * parent->lat;
          rp->lon = (1 - dp) * lon + dp * parent->lon;
          rp->propagated =
              true;  // not a "real" position so we don't propagate it either.
          goto add_position;
        } else
          continue;
      }
    }

    {
      int newpolar = configuration.boat.TrySwitchPolar(
          polar, VW, H, S, configuration.OptimizeTacking);
      if (newpolar == -1) {
        configuration.polar_failed = true;
        continue;
      }
      if (polar == -1) polar = newpolar;

      /* did we tack thru the wind? apply penalty */
      bool tacked = false;
      if (parent_heading * H < 0 && fabs(parent_heading - H) < 180) {
        timeseconds -= configuration.TackingTime;
        tacked = true;
      }

      if (!ComputeBoatSpeed(configuration, timeseconds, twd, tws, W, VW,
                            currentDir, currentSpeed, H, atlas, data_mask, ctw,
                            stw, cog, sog, dist, newpolar))
        continue;

      double dlat, dlon;
      if (configuration.Integrator == RouteMapConfiguration::RUNGE_KUTTA) {
        double k2_dist, k2_BG, k3_dist, k3_BG, k4_dist, k4_BG;
        // a lot more experimentation is needed here, maybe use grib for the
        // right time??
        wxDateTime rk_time_2 =
            configuration.time + wxTimeSpan::Seconds(timeseconds / 2);
        wxDateTime rk_time =
            configuration.time + wxTimeSpan::Seconds(timeseconds);
        if (!rk_step(this, timeseconds, cog, dist / 2, H, configuration,
                     configuration.grib, rk_time_2, newpolar, k2_BG, k2_dist,
                     data_mask) ||
            !rk_step(this, timeseconds, cog, k2_dist / 2, H + k2_BG - cog,
                     configuration, configuration.grib, rk_time_2, newpolar,
                     k3_BG, k3_dist, data_mask) ||
            !rk_step(this, timeseconds, cog, k3_dist, H + k3_BG - cog,
                     configuration, configuration.grib, rk_time, newpolar,
                     k4_BG, k4_dist, data_mask))
          continue;

        ll_gc_ll(lat, lon, cog,
                 dist / 6 + k2_dist / 3 + k3_dist / 3 + k4_dist / 6, &dlat,
                 &dlon);
      } else /* newtons method */
#if 1
        ll_gc_ll(lat, lon, heading_resolve(cog), dist, &dlat, &dlon);
#else
      {
        double d = dist / 60;
        dlat = lat + d * cos(deg2rad(cog));
        dlon = lon + d * sin(deg2rad(cog));
        dlon = heading_resolve(dlon);
      }
#endif

      if (configuration.positive_longitudes && dlon < 0) dlon += 360;

      if (configuration.MaxCourseAngle < 180) {
        double bearing;
        // this is faster than gc distance, and actually works better in higher
        // latitudes
        double d1 = dlat - configuration.StartLat,
               d2 = dlon - configuration.StartLon;
        d2 *= cos(deg2rad(dlat)) / 2;  // correct for latitude
        bearing = rad2deg(atan2(d2, d1));

        if (fabs(heading_resolve(configuration.StartEndBearing - bearing)) >
            configuration.MaxCourseAngle)
          continue;
      }

      if (configuration.MaxDivertedCourse < 180) {
        double bearing, dist;
        double bearing1, dist1;

        double d1 = dlat - configuration.EndLat,
               d2 = dlon - configuration.EndLon;
        d2 *= cos(deg2rad(dlat)) / 2;  // correct for latitude
        bearing = rad2deg(atan2(d2, d1));
        dist = sqrt(pow(d1, 2) + pow(d2, 2));

        d1 = configuration.StartLat - dlat, d2 = configuration.StartLon - dlon;
        bearing1 = rad2deg(atan2(d2, d1));
        dist1 = sqrt(pow(d1, 2) + pow(d2, 2));

        double term = (dist1 + dist) / dist;
        term = pow(term / 16, 4) + 1;  // make 1 until the end, then make big

        if (fabs(heading_resolve(bearing1 - bearing)) >
            configuration.MaxDivertedCourse * term)
          continue;
      }

      /* quick test first to avoid slower calculation */
      if (stw + VW > configuration.MaxApparentWindKnots &&
          Polar::VelocityApparentWind(stw, H, VW) >
              configuration.MaxApparentWindKnots)
        continue;

      if (configuration.DetectLand || configuration.DetectBoundary) {
        double dlat1, dlon1;
        double bearing, dist2end;
        double dist2test;

        // it's not an error if there's boundaries after we reach destination
        ll_gc_ll_reverse(lat, lon, configuration.EndLat, configuration.EndLon,
                         &bearing, &dist2end);
        if (dist2end < dist) {
          dist2test = dist2end;
          ll_gc_ll(lat, lon, heading_resolve(cog), dist2test, &dlat1, &dlon1);
        } else {
          dist2test = dist;
          dlat1 = dlat;
          dlon1 = dlon;
        }

        /* landfall test */
        if (configuration.DetectLand) {
          double ndlon1 = dlon1;

          // Check first if crossing land.
          if (ndlon1 > 360) {
            ndlon1 -= 360;
          }
          if (CrossesLand(dlat1, ndlon1)) {
            configuration.land_crossing = true;
            continue;
          }

          // CUSTOMIZATION - Safety distance from land
          // -----------------------------------------
          // Modify the routing according to a safety
          // margin defined by the user from the land.
          // CONFIG: 2 NM as a security distance by default.
          double distSecure = configuration.SafetyMarginLand;
          double latBorderUp1, lonBorderUp1, latBorderUp2, lonBorderUp2;
          double latBorderDown1, lonBorderDown1, latBorderDown2, lonBorderDown2;

          // Test if land is found within a rectangle with
          // dimensions (dist, distSecure). Tests borders, plus diag,
          // and middle of each side...
          //            <- dist ->
          // |-------------------------------|
          // |                               |    ^
          // |                               |    distSafety
          // |-------------------------------|    ^
          // |                               |
          // |                               |
          // |-------------------------------|

          // Fist, find the (lat,long) of each
          // points of the rectangle
          ll_gc_ll(lat, lon, heading_resolve(cog) - 90, distSecure,
                   &latBorderUp1, &lonBorderUp1);
          ll_gc_ll(dlat1, dlon1, heading_resolve(cog) - 90, distSecure,
                   &latBorderUp2, &lonBorderUp2);
          ll_gc_ll(lat, lon, heading_resolve(cog) + 90, distSecure,
                   &latBorderDown1, &lonBorderDown1);
          ll_gc_ll(dlat1, dlon1, heading_resolve(cog) + 90, distSecure,
                   &latBorderDown2, &lonBorderDown2);

          // Then, test if there is land
          if (PlugIn_GSHHS_CrossesLand(latBorderUp1, lonBorderUp1, latBorderUp2,
                                       lonBorderUp2) ||
              PlugIn_GSHHS_CrossesLand(latBorderDown1, lonBorderDown1,
                                       latBorderDown2, lonBorderDown2) ||
              PlugIn_GSHHS_CrossesLand(latBorderUp1, lonBorderUp1,
                                       latBorderDown2, lonBorderDown2) ||
              PlugIn_GSHHS_CrossesLand(latBorderDown1, lonBorderDown1,
                                       latBorderUp2, lonBorderUp2)) {
            configuration.land_crossing = true;
            continue;
          }
        }

        /* Boundary test */
        if (configuration.DetectBoundary) {
          if (EntersBoundary(dlat1, dlon1)) {
            configuration.boundary_crossing = true;
            continue;
          }
        }
      }
      /* crosses cyclone track(s)? */
      if (configuration.AvoidCycloneTracks &&
          RouteMap::ClimatologyCycloneTrackCrossings) {
        int crossings = RouteMap::ClimatologyCycloneTrackCrossings(
            lat, lon, dlat, dlon, configuration.time,
            configuration.CycloneMonths * 30 + configuration.CycloneDays);
        if (crossings > 0) continue;
      }

      rp = new Position(dlat, dlon, this, H, ctw, newpolar, tacks + tacked,
                        data_mask, configuration.grib_is_data_deficient);
    }
  add_position:

    if (points) {
      rp->prev = points->prev;
      rp->next = points;
      points->prev->next = rp;
      points->prev = rp;
    } else {
      rp->prev = rp->next = rp;
      points = rp;
    }
    count++;
  }

  if (count < 3) { /* would get eliminated anyway, but save the extra steps */
    if (count) DeletePoints(points);
    return false;
  }

  IsoRoute* nr = new IsoRoute(points->BuildSkipList());
  routelist.push_back(nr);
  return true;
}

double RoutePoint::PropagateToPoint(double dlat, double dlon,
                                    RouteMapConfiguration& configuration,
                                    double& H, int& data_mask, bool end) {
  double S = Swell(configuration, lat, lon);
  if (S > configuration.MaxSwellMeters) return NAN;

  if (fabs(lat) > configuration.MaxLatitude) return NAN;

  double twd, tws, W, VW, currentDir, currentSpeed;
  climatology_wind_atlas atlas;
  if (!ReadWindAndCurrents(configuration, this, twd, tws, W, VW, currentDir,
                           currentSpeed, atlas, data_mask)) {
    if (!end) configuration.wind_data_failed = true;
    return NAN;
  }

  if (VW > configuration.MaxTrueWindKnots) return NAN;

  /* todo: we should make sure we don't tack if we are already at the max tacks,
     possibly perform other tests and/or switch sail polar? */
  double bearing, dist;
  ll_gc_ll_reverse(lat, lon, dlat, dlon, &bearing, &dist);

  /* figure out bearing and distance to go, because it is a non-linear problem
     if compounded with currents solve iteratively (without currents only one
     iteration will ever occur */
  double ctw, stw, cog = W, sog;
  int iters = 0;
  H = 0;
  int newpolar = polar;
  bool old = configuration.OptimizeTacking;
  if (end) configuration.OptimizeTacking = true;
  do {
    /* make our correction in range */
    while (bearing - cog > 180) bearing -= 360;
    while (cog - bearing > 180) bearing += 360;

    H += bearing - cog;
    ctw = W + H; /* rotated relative to true wind */

    double dummy_dist;  // not used

    newpolar = configuration.boat.TrySwitchPolar(polar, VW, H, S,
                                                 configuration.OptimizeTacking);
    if (newpolar == -1) {
      configuration.polar_failed = true;
      configuration.OptimizeTacking = old;
      return NAN;
    }

    if (!ComputeBoatSpeed(configuration, 0, twd, tws, W, VW, currentDir,
                          currentSpeed, H, atlas, data_mask, ctw, stw, cog, sog,
                          dummy_dist, newpolar) ||
        ++iters == 10  // give up
    ) {
      configuration.OptimizeTacking = old;
      return NAN;
    }
  } while ((bearing - cog) > 1e-3);
  configuration.OptimizeTacking = old;

  /* only allow if we fit in the isochron time.  We could optimize this by
     finding the maximum boat speed once, and using that before computing boat
     speed for this angle, but for now, we don't worry because propagating to
     the end is a small amount of total computation */
  if (end && dist / sog > configuration.UsedDeltaTime / 3600.0) return NAN;

  /* quick test first to avoid slower calculation */
  if (stw + VW > configuration.MaxApparentWindKnots &&
      Polar::VelocityApparentWind(stw, H, VW) >
          configuration.MaxApparentWindKnots)
    return NAN;

  /* landfall test if we are within 60 miles (otherwise it's very slow) */
  if (configuration.DetectLand && dist < 60 && CrossesLand(dlat, dlon)) {
    if (!end) configuration.land_crossing = true;
    return NAN;
  }

  /* Boundary test */
  if (configuration.DetectBoundary && EntersBoundary(dlat, dlon)) {
    if (!end) configuration.boundary_crossing = true;
    return NAN;
  }

  /* crosses cyclone track(s)? */
  if (configuration.AvoidCycloneTracks &&
      RouteMap::ClimatologyCycloneTrackCrossings) {
    int crossings = RouteMap::ClimatologyCycloneTrackCrossings(
        lat, lon, configuration.EndLat, configuration.EndLon,
        configuration.time,
        configuration.CycloneMonths * 30 + configuration.CycloneDays);

    if (crossings > 0) return NAN;
  }
  polar = newpolar;

  return 3600.0 * dist / sog;
}

double Position::Distance(Position* p) {
  return DistGreatCircle(lat, lon, p->lat, p->lon);
}

bool RoutePoint::CrossesLand(double dlat, double dlon) {
  return PlugIn_GSHHS_CrossesLand(lat, lon, dlat, dlon);
}

int Position::SailChanges() {
  if (!parent) return 0;

  return (polar != parent->polar) + parent->SailChanges();
}

bool RoutePoint::EntersBoundary(double dlat, double dlon) {
  struct FindClosestBoundaryLineCrossing_t t;
  t.dStartLat = lat, t.dStartLon = heading_resolve(lon);
  t.dEndLat = dlat, t.dEndLon = heading_resolve(dlon);
  t.sBoundaryState = wxT("Active");

  // we request any type
  return RouteMap::ODFindClosestBoundaryLineCrossing(&t);
}

SkipPosition::SkipPosition(Position* p, int q) : point(p), quadrant(q) {}

void SkipPosition::Remove() {
  prev->next = next;
  next->prev = prev;
  delete this;
}

/* copy a skip list along with it's position list to new lists */
SkipPosition* SkipPosition::Copy() {
  SkipPosition* s = this;
  if (!s) return s;

  SkipPosition *fs, *ns = NULL;
  Position *fp, *np = NULL;
  Position* p = s->point;
  do {
    Position* nsp = NULL;
    do { /* copy all positions between skip positions */
      Position* nnp = new Position(p);
      if (!nsp) nsp = nnp;
      if (np) {
        np->next = nnp;
        nnp->prev = np;
        np = nnp;
      } else {
        fp = np = nnp;
        np->prev = np->next = np;
      }
      p = p->next;
    } while (p != s->next->point);

    SkipPosition* nns = new SkipPosition(nsp, s->quadrant);
    if (ns) {
      ns->next = nns;
      nns->prev = ns;
      ns = nns;
    } else {
      fs = ns = nns;
      ns->prev = ns->next = nns;
    }
    s = s->next;
  } while (s != this);

  fs->prev = ns;
  ns->next = fs;

  fp->prev = np;
  np->next = fp;
  return fs;
}

void DeleteSkipPoints(SkipPosition* skippoints) {
  SkipPosition* s = skippoints;
  do {
    SkipPosition* ds = s;
    s = s->next;
    delete ds;
  } while (s != skippoints);
}

IsoRoute::IsoRoute(SkipPosition* s, int dir)
    : skippoints(s), direction(dir), parent(NULL) {
  /* make sure the skip points start at the minimum
     latitude so we know we are on the outside */
  MinimizeLat();
}

/* copy constructor */
IsoRoute::IsoRoute(IsoRoute* r, IsoRoute* p)
    : skippoints(r->skippoints->Copy()), direction(r->direction), parent(p) {}

IsoRoute::~IsoRoute() {
  for (IsoRouteList::iterator it = children.begin(); it != children.end(); ++it)
    delete *it;

  if (!skippoints) return;

  DeletePoints(skippoints->point);
  DeleteSkipPoints(skippoints);
}

void IsoRoute::Print() {
  if (!skippoints)
    printf("Empty IsoRoute\n");
  else {
    Position* p = skippoints->point;
    do {
      printf("%.10f %.10f\n", p->lon, p->lat);
      p = p->next;
    } while (p != skippoints->point);
    printf("\n");
  }
}

void IsoRoute::PrintSkip() {
  if (!skippoints)
    printf("Empty IsoRoute\n");
  else {
    SkipPosition* s = skippoints;
    do {
      printf("%.10f %.10f\n", s->point->lon, s->point->lat);
      s = s->next;
    } while (s != skippoints);
    printf("\n");
  }
}

void IsoRoute::MinimizeLat() {
  SkipPosition *min = skippoints, *cur = skippoints;
  do {
    if (cur->point->lat < min->point->lat) min = cur;
    cur = cur->next;
  } while (cur != skippoints);
  skippoints = min;
}

/* how many times do we cross this route going from this point to infinity,
   return -1 if inconclusive */
int IsoRoute::IntersectionCount(Position& pos) {
  int numintsct = 0;
  double lat = pos.lat, lon = pos.lon;

  SkipPosition* s1 = skippoints;

  double s1plon = s1->point->lon;
  int state1 = (lon < s1plon);
  do {
    SkipPosition* s2 = s1->next;
    double s2plon = s2->point->lon;
    int state0 = state1;
    state1 = (lon < s2plon);
    if (state0 != state1) {
      double s1plat = s1->point->lat, s2plat = s2->point->lat;
      int state = (lat < s1plat) + (lat < s2plat);

      switch (state) {
        case 1: /* must test every point in this case as point falls in
                   boundaries of skip list */
        {
          Position* p1 = s1->point;
          double p1lon = p1->lon;
          int pstate1 = lon < p1lon;
          do {
            Position* p2 = p1->next;
            double p2lon = p2->lon;
            int pstate0 = pstate1;
            pstate1 = lon < p2lon;

#if 1
            if (lon == p1lon && lon == p2lon)
              printf("degenerate case not handled properly\n");
#endif

            if (pstate0 != pstate1) {
              double p1lat = p1->lat, p2lat = p2->lat;
              state = (lat < p1lat) + (lat < p2lat);
              switch (state) {
                case 1: /* must perform exact intersection test */
                {
                  double p1lon = p1->lon;
#if 0
                            int dir = TestIntersectionXY(p1lon, p1lat, p2lon, p2lat, lon, lat, lon, 91);
                            switch(dir) {
                            case -2: case -3: case 2: case 3: return -1;
                            case 1: case -1: goto intersects;
                            }
#else
                  double m1 = (lat - p1lat) * (p2lon - p1lon);
                  double m2 = (lon - p1lon) * (p2lat - p1lat);

                  if (s1->quadrant & 1) {
                    if (m1 < m2) goto intersects;
                  } else if (m1 > m2)
                    goto intersects;
#endif
                } break;
                case 2: /* must intersect, we are below */
                  goto intersects;
              }
            }
            p1 = p2;
          } while (p1 != s2->point);
        } break;
        case 2: /* must intersect, we are below skip segment out of bounds */
        intersects:
          numintsct++;
      }
    }

    s1 = s2;
  } while (s1 != skippoints);

  return numintsct;
}

/* determine if a route contains a position
   0 for outside, 1 for inside, -1 for inconclusive (on border or really close)
 */
int IsoRoute::Contains(Position& pos, bool test_children) {
  int numintsct = IntersectionCount(pos);
  if (numintsct == -1) return -1;

  if (test_children)
    for (IsoRouteList::iterator it = children.begin(); it != children.end();
         it++) {
      int cnumintsct = (*it)->Contains(pos, test_children);
      if (cnumintsct == -1) return -1;
      numintsct += cnumintsct;
    }

  return numintsct & 1; /* odd */
}

/* This function is very slow, and should probably be removed
   or replaced with something else.. see how often it is called */
bool IsoRoute::CompletelyContained(IsoRoute* r) {
  Position* pos = r->skippoints->point;
  do {
    if (Contains(*pos, false) != 1) return false;
    pos = pos->next;
  } while (pos != r->skippoints->point);
  return true;
}

/* Determine if one route contains another,
   only test first point, but if it fails try other points */
bool IsoRoute::ContainsRoute(IsoRoute* r) {
  Position* pos = r->skippoints->point;
  do {
    switch (Contains(*pos, false)) {
      case 0:
        return false;
      case 1:
        return true;
    }

    pos = pos->next;
  } while (pos !=
           r->skippoints
               ->point); /* avoid deadlock.. lets hope we dont do this often */
  //    printf("bad contains route\n");
  return true; /* probably good to say it is contained in this unlikely case */
}

/* remove points which are right next to eachother on the graph to speed
   computation time */
void IsoRoute::ReduceClosePoints() {
  const double eps = 2e-5; /* resolution of 2 meters should be sufficient */
  Position* p = skippoints->point;
  while (p != skippoints->point->prev) {
    Position* n = p->next;
    double dlat = p->lat - n->lat, dlon = p->lon - n->lon;
    if (fabs(dlat) < eps && fabs(dlon) < eps) {
      p->next = n->next;
      n->next->prev = p;
      delete n;
    } else
      p = n;
  }

  DeleteSkipPoints(skippoints);
  skippoints = p->BuildSkipList();

  for (IsoRouteList::iterator it = children.begin(); it != children.end(); it++)
    (*it)->ReduceClosePoints();
}

/* apply current to given route, and return if it changed at all */
#if 0
bool IsoRoute::ApplyCurrents(GribRecordSet *grib, wxDateTime time, RouteMapConfiguration &configuration)
{
    if(!skippoints)
        return false;

    bool ret = false;
    Position *p = skippoints->point;
    double timeseconds = configuration.UsedDeltaTime;
    do {
        double currentDir, currentSpeed;
        if(configuration.Currents && Current(grib, configuration.ClimatologyType,
                                             time, p->lat, p->lon, currentDir, currentSpeed)) {
            /* drift distance over ground */
            double dist = currentSpeed * timeseconds / 3600.0;
            if(dist)
                ret = true;
            ll_gc_ll(p->lat, p->lon, currentDir, dist, &p->lat, &p->lon);
        }

        p = p->next;
    } while(p != skippoints->point);

    /* if we moved we need to rebuild the skip list */
    if(ret) {
        Position *points = skippoints->point;
        DeleteSkipPoints(skippoints);
        skippoints = points->BuildSkipList();
    }

    return ret;
}
#endif

enum { MINLON, MAXLON, MINLAT, MAXLAT };
/* return false if longitude is possibly invalid
   could cache these bounds to avoid recomputing all the time */
void IsoRoute::FindIsoRouteBounds(double bounds[4]) {
  SkipPosition* maxlat = skippoints;
  Position* p = skippoints->point;
  bounds[MINLAT] = bounds[MAXLAT] = p->lat;
  bounds[MINLON] = bounds[MAXLON] = p->lon;

  SkipPosition* s = skippoints->next;
  while (s != skippoints) {
    p = s->point;
    bounds[MINLAT] = wxMin(p->lat, bounds[MINLAT]);
    bounds[MAXLAT] = wxMax(p->lat, bounds[MAXLAT]);
    bounds[MINLON] = wxMin(p->lon, bounds[MINLON]);
    bounds[MAXLON] = wxMax(p->lon, bounds[MAXLON]);

    if (p->lat == bounds[MAXLAT]) maxlat = s;
    s = s->next;
  }
  skippoints = maxlat; /* set to max lat for merging to keep outside */
}

bool checkskiplist(SkipPosition* s) {
  /* build skip list of positions, skipping over strings of positions in
     the same quadrant */
  SkipPosition* skippoints = s;
  Position* p = s->point;
  do {
    do {
      Position* q = p->next;
      int quadrant = ComputeQuadrantFast(p, q);

      if (quadrant != s->quadrant) return false;
      p = q;
    } while (p != s->next->point);
    s = s->next;
  } while (s != skippoints);
  return true;
}

/* remove and delete a position given it's last skip position,
   we need to update the skip list if this point falls on a skip position*/
void IsoRoute::RemovePosition(SkipPosition* s, Position* p) {
  p->next->prev = p->prev;
  p->prev->next = p->next;

  if (s->point == p) {
    if (s == s->next) {
      delete s;
      skippoints = NULL;
    } else {
      /* rebuild skip list */
      Position* points = skippoints->point;
      if (p == points) points = points->next;
      DeleteSkipPoints(skippoints);
      skippoints = points->BuildSkipList();
      /* make sure the skip points start at the minimum
         latitude so we know we are on the outside */
      MinimizeLat();
    }
  }
  delete p;
}

inline void SwapSegments(Position* p, Position* q, Position* r, Position* s) {
  p->next = s;
  s->prev = p;
  r->next = q;
  q->prev = r;
}

inline void SwapSkipSegments(SkipPosition* sp, SkipPosition* sq,
                             SkipPosition* sr, SkipPosition* ss) {
  sp->next = ss;
  ss->prev = sp;
  sr->next = sq;
  sq->prev = sr;
}

inline void InsertSkipPosition(SkipPosition* sp, SkipPosition* sn, Position* p,
                               int quadrant) {
  SkipPosition* s = new SkipPosition(p, quadrant);
  s->prev = sp;
  sp->next = s;
  s->next = sn;
  sn->prev = s;
}

/* given positions p and s in skip list between sp and ss, fix stuff adding
   removing or shifting skip positions to make things valid after this merge */
/*inline*/ void FixSkipList(SkipPosition* sp, SkipPosition* ss, Position* p,
                            Position* s, int rquadrant, SkipPosition*& spend,
                            SkipPosition*& ssend) {
  int quadrant = ComputeQuadrantFast(p, s);
  if (sp->point == p) {
    sp->quadrant = quadrant; /* reuse p with this quadrant */

    if (quadrant == sp->prev->quadrant && sp != ss) {
      sp->point = sp->prev->point;
      if (sp->prev == spend) spend = sp;
      if (sp->prev == ssend) ssend = sp;
      if (ss == sp->prev) {
        if (ssend == ss) ssend = sp;
        ss = sp;
      }
      sp->prev->Remove();
    }
    /* DUPLICATE START */
    if (quadrant == rquadrant) {
      if (rquadrant == ss->quadrant) goto remove;
    } else if (ss->point == s) {
      if (quadrant == ss->quadrant) goto remove;
    } else {
      if (rquadrant == ss->quadrant)
        ss->point = s; /* shift ss to s */
      else
        InsertSkipPosition(sp, ss, s, rquadrant);
    }
    /* DUPLICATE END */
  } else if (sp->quadrant == quadrant) {
    if (quadrant ==
        rquadrant) { /* this is never hit..  can we remove this test? */
      if (rquadrant == ss->quadrant) goto remove;
    } else if (ss->point == s) {
      if (quadrant == ss->quadrant) {
      remove:
        if (sp == ss) printf("sp == ss.. this is bad\n");
        if (ss == spend) spend = ss->next;
        if (ss == ssend) ssend = ss->next;
        ss->Remove();
      }
    } else {
      if (rquadrant == ss->quadrant)
        ss->point = s; /* shift ss to s */
      else
        InsertSkipPosition(sp, ss, s, rquadrant);
    }
  } else {
    if (quadrant == rquadrant) {
      if (rquadrant == ss->quadrant)
        ss->point = p; /* shift ss to p */
      else
        InsertSkipPosition(sp, ss, p, quadrant);
    } else if (ss->point == s) {
      if (quadrant == ss->quadrant)
        ss->point = p; /* shift ss to p */
      else
        InsertSkipPosition(sp, ss, p, quadrant);
    } else {
      InsertSkipPosition(sp, ss, p, quadrant);
      if (rquadrant == ss->quadrant)
        ss->point = s; /* shift ss to s */
      else
        InsertSkipPosition(sp->next, ss, s, rquadrant);
    }
  }
}

bool UpdateEnd(SkipPosition* spend, SkipPosition* sr) {
  SkipPosition* nsr = sr;
  do {
    if (nsr == spend) return true;
    nsr = nsr->next;
  } while (nsr != sr);
  return false;
}

#define COMPUTE_MIN_MAX(quadrant, A, B, N) \
  switch (quadrant) {                      \
    default:                               \
      min##N##x = B##x;                    \
      max##N##x = A##x;                    \
      min##N##y = B##y, max##N##y = A##y;  \
      break;                               \
    case 1:                                \
      min##N##x = A##x;                    \
      max##N##x = B##x;                    \
      min##N##y = B##y, max##N##y = A##y;  \
      break;                               \
    case 2:                                \
      min##N##x = B##x;                    \
      max##N##x = A##x;                    \
      min##N##y = A##y, max##N##y = B##y;  \
      break;                               \
    case 3:                                \
      min##N##x = A##x;                    \
      max##N##x = B##x;                    \
      min##N##y = A##y, max##N##y = B##y;  \
      break;                               \
  }

#define COMPUTE_STATE(state, S, N)    \
  state = 0;                          \
  if (S##x >= min##N##x) state += 4;  \
  if (S##x > max##N##x) state += 4;   \
  if (S##y >= min##N##y) state += 12; \
  if (S##y > max##N##y) state += 12;

/* 0 1    0  4  8
   2 3   12 16 20
         24 28 32 */
#define UPDATE_STATE(state, quadrant, skip, S, N)     \
  switch (state + quadrant) {                         \
    case 1:                                           \
      if (S##x >= min##N##x) {                        \
        skip##c1 : if (S##x > max##N##x) state = 8;   \
        else state = 4;                               \
      } /*f*/                                         \
    case 0:                                           \
      goto skip;                                      \
    case 3:                                           \
      if (S##x >= min##N##x) {                        \
        if (S##y >= min##N##y) break;                 \
        goto skip##c1;                                \
      } /*f*/                                         \
    case 2:                                           \
      if (S##y >= min##N##y) {                        \
        if (S##y > max##N##y)                         \
          state = 24;                                 \
        else                                          \
          state = 12;                                 \
      }                                               \
      goto skip;                                      \
                                                      \
    case 6:                                           \
      if (S##y >= min##N##y) break; /*f*/             \
    case 4:                                           \
      if (S##x < min##N##x) state = 0;                \
      goto skip;                                      \
    case 7:                                           \
      if (S##y >= min##N##y) break; /*f*/             \
    case 5:                                           \
      if (S##x > max##N##x) state = 8;                \
      goto skip;                                      \
                                                      \
    case 8:                                           \
      if (S##x <= max##N##x) {                        \
        skip##c8 : if (S##x < min##N##x) state = 0;   \
        else state = 4;                               \
      } /*f*/                                         \
    case 9:                                           \
      goto skip;                                      \
    case 10:                                          \
      if (S##x <= max##N##x) {                        \
        if (S##y >= min##N##y) break;                 \
        goto skip##c8;                                \
      } /*f*/                                         \
    case 11:                                          \
      if (S##y >= min##N##y) {                        \
        if (S##y > max##N##y)                         \
          state = 32;                                 \
        else                                          \
          state = 20;                                 \
      }                                               \
      goto skip;                                      \
                                                      \
    case 13:                                          \
      if (S##x >= min##N##x) break; /*f*/             \
    case 12:                                          \
      if (S##y < min##N##y) state = 0;                \
      goto skip;                                      \
    case 15:                                          \
      if (S##x >= min##N##x) break; /*f*/             \
    case 14:                                          \
      if (S##y > max##N##y) state = 24;               \
      goto skip;                                      \
      /* 16-19 fall through */                        \
    case 20:                                          \
      if (S##x <= max##N##x) break; /*f*/             \
    case 21:                                          \
      if (S##y < min##N##y) state = 8;                \
      goto skip;                                      \
    case 22:                                          \
      if (S##x <= max##N##x) break; /*f*/             \
    case 23:                                          \
      if (S##y > max##N##y) state = 32;               \
      goto skip;                                      \
                                                      \
    case 25:                                          \
      if (S##x >= min##N##x) {                        \
        if (S##y <= max##N##y) break;                 \
        goto skip##c27;                               \
      } /*f*/                                         \
    case 24:                                          \
      if (S##y <= max##N##y) {                        \
        if (S##y < min##N##y)                         \
          state = 0;                                  \
        else                                          \
          state = 12;                                 \
      }                                               \
      goto skip;                                      \
    case 27:                                          \
      if (S##x >= min##N##x) {                        \
        skip##c27 : if (S##x > max##N##x) state = 32; \
        else state = 28;                              \
      } /*f*/                                         \
    case 26:                                          \
      goto skip;                                      \
                                                      \
    case 28:                                          \
      if (S##y <= max##N##y) break; /*f*/             \
    case 30:                                          \
      if (S##x < min##N##x) state = 24;               \
      goto skip;                                      \
    case 29:                                          \
      if (S##y <= max##N##y) break; /*f*/             \
    case 31:                                          \
      if (S##x > max##N##x) state = 32;               \
      goto skip;                                      \
                                                      \
    case 32:                                          \
      if (S##x <= max##N##x) {                        \
        if (S##y <= max##N##y) break;                 \
        goto skip##c34;                               \
      } /*f*/                                         \
    case 33:                                          \
      if (S##y <= max##N##y) {                        \
        if (S##y < min##N##y)                         \
          state = 8;                                  \
        else                                          \
          state = 20;                                 \
      }                                               \
      goto skip;                                      \
    case 34:                                          \
      if (S##x <= max##N##x) {                        \
        skip##c34 : if (S##x < min##N##x) state = 24; \
        else state = 28;                              \
      } /*f*/                                         \
    case 35:                                          \
      goto skip;                                      \
  }

/* This function is the heart of the route map algorithm.
   Essentially search for intersecting line segments, and flip them correctly
   while maintaining a skip list.
 */
bool Normalize(IsoRouteList& rl, IsoRoute* route1, IsoRoute* route2, int level,
               bool inverted_regions) {
  bool normalizing;

reset:
  SkipPosition *spend = route1->skippoints, *ssend = route2->skippoints;

  if (!spend || spend->prev == spend->next) { /* less than 3 items */
    delete route1;
    if (route1 != route2) rl.push_back(route2);
    return true;
  }

  if (route1 == route2) {
    normalizing = true;
  } else {
    if (!ssend || ssend->prev == ssend->next) { /* less than 3 items */
      delete route2;
      if (spend) rl.push_back(route1);
      return true;
    }

    normalizing = false;
  }

  SkipPosition* sp = spend;
startnormalizing:
  do {
    SkipPosition* sq = sp->next;
    SkipPosition *sr, *ss;
    if (normalizing)
      ss = sp;
    else
      ss = ssend;

    Position *p = sp->point, *q = sq->point;
    double px = p->lon, qx = q->lon, py = p->lat, qy = q->lat;

    double minx, maxx, miny, maxy;
    COMPUTE_MIN_MAX(sp->quadrant, p, q, )

    Position *r, *s = ss->point;

    int dir;
    double rx, ry;
    double sx = s->lon, sy = s->lat;

    int state, rstate, pstate;
    COMPUTE_STATE(state, s, )

    int nr;
    Position *pstart, *pend, *rstart, *rend;

    do {
      sr = ss;
      ss = sr->next;

      s = ss->point;
      sx = s->lon, sy = s->lat;

      UPDATE_STATE(state, sr->quadrant, skip, s, )

      nr = 0;
      if (normalizing) {
        if (sp == sr) {
          nr = 1; /* only occurs during normalizing (first round) */
          /* normalizing and overlapping round.. don't bother to calculate
           * smaller bounds */
          pstart = sp->point;
          pend = sq->point;

          rstart = sr->point;
          rend = ss->point;
          goto skip_bounds_compute;
        } else if (sq == sr)
          nr = 2; /* only occurs normalizing (second round) */
        else if (ss == sp)
          nr = 3; /* only occurs normalizing (last round) */
      }

#if 1 /* this is only slightly faster, barely can measure a difference */
      /* compute bounds for these skip segments */
      double minrx, maxrx, minry, maxry;
      rx = sr->point->lon, ry = sr->point->lat;
      COMPUTE_MIN_MAX(sr->quadrant, r, s, r)

      pstart = pend = NULL;
      q = sp->point;
      qx = q->lon, qy = q->lat;
      COMPUTE_STATE(pstate, q, r)
      do {
        p = q;
        q = q->next;
        qx = q->lon, qy = q->lat;
        UPDATE_STATE(pstate, sp->quadrant, skipp, q, r)
        if (!pstart) pstart = p;
        pend = q;
        COMPUTE_STATE(pstate, q, r)
        goto startingp;
      skipp:
        if (pstart) break; /* have start, must be done */
      startingp:;
      } while (q != sq->point);
      p = pstart;
      if (!pstart) goto done;
      //    if(pstart == pend)  // this is never hit in practice
      //      goto done;

      rstart = rend = NULL;
      s = sr->point;
      rstate = state; /* still valid from before */
      do {
        r = s;
        s = s->next;
        sx = s->lon, sy = s->lat;
        UPDATE_STATE(rstate, sr->quadrant, skipr, s, )
        if (!rstart) rstart = r;
        rend = s;
        COMPUTE_STATE(rstate, s, )
        goto startingr;
      skipr:
        if (rstart) break; /* have start, must be done */
      startingr:;
      } while (s != ss->point);

      if (!rstart) goto done;
#else
      pstart = sp->point;
      pend = sq->point;

      rstart = sr->point;
      rend = ss->point;
#endif
    skip_bounds_compute:

      p = pstart;
      do {
        q = p->next;

        switch (nr) {
          case 1:
            s = q;
            if (s == rend) goto done;
            s = s->next;
            break;
          case 2:
            s = rstart;
            if (s == q) s = s->next;
            break;
          case 3:
            s = rstart;
            if (rend == p) rend = rend->prev;
            break;
          default:
            s = rstart;
        }

        if (s == rend) goto done;

        px = p->lon, py = p->lat;
        qx = q->lon, qy = q->lat;

        double minpqx, maxpqx, minpqy, maxpqy;
        COMPUTE_MIN_MAX(sp->quadrant, p, q, pq)

        sx = s->lon, sy = s->lat;
        COMPUTE_STATE(state, s, pq);
        do {
          r = s;
          s = r->next;

          sx = s->lon, sy = s->lat;
          UPDATE_STATE(state, sr->quadrant, skippr, s, pq);

          rx = r->lon, ry = r->lat;
          dir = TestIntersectionXY(px, py, qx, qy, rx, ry, sx, sy);
          switch (dir) {
            case -2:
              route1->skippoints = spend, route2->skippoints = ssend;
              route1->RemovePosition(sp, p);
              goto reset;
            case -3:
              route1->skippoints = spend, route2->skippoints = ssend;
              route1->RemovePosition(sq, q);
              goto reset;
            case 2:
              route1->skippoints = spend, route2->skippoints = ssend;
              route2->RemovePosition(sr, r);
              goto reset;
            case 3:
              route1->skippoints = spend, route2->skippoints = ssend;
              route2->RemovePosition(ss, s);
              goto reset;
            case -1:
            case 1:
              if (!normalizing) { /* sanity check for merging */
                if (dir == -1) {
                  if (route1->direction != 1 || route2->direction != -1)
                    /* we intersected at the wrong side, skip this intersection
                       and continue to find the intersection we want,  this
                       occurs when a line segment passes completely through a
                       region. We could possibly merge here anyway but the
                       result would be less correct.  */
                    goto skipmerge;
                } else
                  /* inverted invalid test */
                  if (route1->direction == 1 && route2->direction == -1)
                    goto skipmerge;
              } else {
                if (level == 0 && dir == -1 && route1->direction == 1)
                  goto skipmerge;
              }

              SwapSegments(p, q, r, s);         /* update position list */
              SwapSkipSegments(sp, sq, sr, ss); /* update skip lists */

              /* now update skip list properly */
              Position* orig_sppoint = sp->point;
              if (sp->quadrant != sr->quadrant) {
                int rquadrant = sr->quadrant, pquadrant = sp->quadrant;
                FixSkipList(sp, ss, p, s, rquadrant, spend, ssend);
                FixSkipList(sr, sq, r, q, pquadrant, spend, ssend);
              }

              if (normalizing) {
                /* did the end end up in the subroute? move it back out */
                if (UpdateEnd(spend, sr)) spend = sp->next;
                if (UpdateEnd(ssend, sr)) ssend = sp->next;

                if (level == 0) {
                  /* slight numerical error, or outer inversion */
                  if (dir != route1->direction || sr->next->next == sr) {
                    DeletePoints(r);
                    DeleteSkipPoints(sr);
                  } else {
                    IsoRoute* x = new IsoRoute(sr, dir);
                    IsoRouteList sub;
                    Normalize(sub, x, x, level + 1, inverted_regions);
                    if (inverted_regions) {
                      for (IsoRouteList::iterator it = sub.begin();
                           it != sub.end(); ++it) {
                        if (!(*it)->children.empty()) {
                          printf("grandchild detected\n");
                          delete *it;
                        } else if (route1->direction == (*it)->direction) {
                          rl.push_back(*it); /* sibling */
                        } else if ((*it)->Count() < 24) {
                          //                      printf("too small to be a
                          //                      useful child: %d\n",
                          //                      (*it)->Count());
                          delete *it;
                        } else if (!(route1->skippoints = spend,
                                     route1->CompletelyContained(*it))) {
                          //                      printf("not correct to be
                          //                      child: %d\n", (*it)->Count());
                          delete *it;
                        } else { /* different direction contained.. it is a
                                    child */
                          /* we should merge it with the other children here */
                          //                      printf("Child route: %d\n",
                          //                      (*it)->Count());
                          IsoRoute* child = *it;
                          child->parent = route1;
                          route1->children.push_back(child);
                        }
                      }
                    } else { /* no inverted regions mode */
                      for (IsoRouteList::iterator it = sub.begin();
                           it != sub.end(); ++it) {
                        if (route1->direction == (*it)->direction) {
                          rl.push_back(*it); /* sibling */
                        } else
                          delete *it; /* inversion */
                      }
                    }
                  }
                } else { /* all subregions are siblings for inner levels */

                  if (sr->next->next ==
                      sr) { /* slight numerical error, or outer inversion */
                    DeletePoints(r);
                    DeleteSkipPoints(sr);
                  } else {
                    IsoRoute* x = new IsoRoute(sr, dir);
                    IsoRouteList sub;
                    Normalize(sub, x, x, level + 1, inverted_regions);
                    rl.splice(rl.end(), sub);
                  }
                }
              } else { /* merging */
                for (IsoRouteList::iterator it = route2->children.begin();
                     it != route2->children.end(); it++)
                  (*it)->parent = route1;

                /* merge children (append is currently incorrect)
                   the children need to be merged, and any overlapping regions
                   incremented so they don't get removed if contained */
                int sc1 = route1->children.size();
                int sc2 = route2->children.size();
                if (sc1 && sc2) printf("both have children: %d %d\n", sc1, sc2);

                route1->children.splice(route1->children.end(),
                                        route2->children);
                route2->skippoints = NULL; /* all points are now in route1 */
                delete route2;
                route2 = route1;
                ssend = spend;
                spend = sr->next; /* after old sq we are done.. this is known */
                /* continue from here and begin to normalize */
#if 0 /* these only needed if we could jump back in too a more optimal spot \
         than startnormalizing */
            /*  could in theory somehow skip to p for this round instead of starting
                at sp->point.. but I doubt it would speed things up that much. */
            sr = sp, ss = sr->next;
            pend = rend = ss->point;
#endif
                normalizing = true;
              }

              if (sp->point != orig_sppoint) {
                /* it is possible we are no longer on the outside
                   because of the skip list getting contracted
                   so we must minimize the latitude at the start of the skiplist
                 */
                route1->skippoints = sp;
                route1->MinimizeLat();
                goto reset;
              }

              goto startnormalizing;
          }
        skipmerge:
          COMPUTE_STATE(state, s, pq);
        skippr:;
        } while (s != rend);
        p = q;
      } while (p != pend);
    done:
      COMPUTE_STATE(state, s, )
    skip:;
    } while (ss != ssend);
    sp = sq;
  } while (sp != spend);

  if (normalizing) {
    route1->skippoints = spend;

    /* make sure the skip points start at the minimum
       latitude so we know we are on the outside */
    route1->MinimizeLat();
    rl.push_back(route1);
    return true;
  }
  return false;
}

/* take two routes that may overlap, and combine into a list of non-overlapping
 * routes */
bool Merge(IsoRouteList& rl, IsoRoute* route1, IsoRoute* route2, int level,
           bool inverted_regions) {
  if (route1->direction == -1 && route2->direction == -1) {
    printf("cannot merge two inverted routes\n");
    exit(1);
  }

  /* quick test to make sure we could possibly intersect with bounds */
  double bounds1[4], bounds2[4];
  route1->FindIsoRouteBounds(bounds1);
  route2->FindIsoRouteBounds(bounds2);
  if (bounds1[MINLAT] > bounds2[MAXLAT] || bounds1[MAXLAT] < bounds2[MINLAT] ||
      bounds1[MINLON] > bounds2[MAXLON] || bounds1[MAXLON] < bounds2[MINLON])
    return false;

  /* make sure route1 is on the outside */
  if (route2->skippoints->point->lat > route1->skippoints->point->lat) {
    IsoRoute* t = route1;
    route1 = route2;
    route2 = t;
  }

  if (Normalize(rl, route1, route2, level, inverted_regions)) return true;

  /* no intersection found, test if the second route is completely
     inside the first */
  if (route1->ContainsRoute(route2)) {
    if (inverted_regions) {
      if (route1->direction == 1 && route2->direction == 1) {
        /* if both region have children, they should get merged
           correctly here instead of this */
        // int sc1 = route1->children.size();
        // int sc2 = route2->children.size();
        // if(sc1 && sc2)
        // printf("both have children contains: %d %d\n", sc1, sc2);

        /* remove all of route2's children, route1 clears them
           (unless they interected with route1 children which we don't handle
           yet */
        for (IsoRouteList::iterator it2 = route2->children.begin();
             it2 != route2->children.end(); it2++)
          delete *it2;
        route2->children.clear();

        /* now determine if route2 affects any of route1's children,
           if there are any intersections, it should mask away that area.
           once completely merged, all the masks are removed and children
           remain */
        IsoRouteList childrenmask;   /* non-inverted */
        IsoRouteList mergedchildren; /* inverted */
        childrenmask.push_back(route2);
        while (!childrenmask.empty()) {
          IsoRoute* r1 = childrenmask.front();
          childrenmask.pop_front();
          while (!route1->children.empty()) {
            IsoRoute* r2 = route1->children.front();
            route1->children.pop_front();
            IsoRouteList childrl; /* see if there is a merge */

            if (Merge(childrl, r1, r2, 1, true)) {
              for (IsoRouteList::iterator cit = childrl.begin();
                   cit != childrl.end(); cit++)
                if ((*cit)->direction == route1->direction)
                  childrenmask.push_back(*cit);
                else {
                  IsoRoute* child = *cit;
                  child->parent = route1;
                  route1->children.push_back(child);
                }
              goto remerge_children;
            } else
              mergedchildren.push_back(r2);
          }
          delete r1; /* all children have been tried, so done with this mask */

        remerge_children:
          route1->children.splice(route1->children.end(), mergedchildren);
        }
      } else if (route1->direction == -1 && route2->direction == -1) {
        delete route1; /* keep smaller region if both inverted */
        route1 = route2;
      } else if (route1->direction == 1 && route2->direction == -1) {
        delete route2;
      } else {
        /* this is a child route with a normal route completely inside..
           a contrived situation it is, should not get here often */
        //                printf("contrived delete: %d, %d\n", route1->Count(),
        //                route2->Count());
        delete route2;
      }
    } else           /* no inverted regions mode */
      delete route2; /* it covers a sub area, delete it */

    rl.push_back(route1); /* no need to normalize */
    return true;
  }

  /* routes close enough to pass initial rectangle test but no
     actual intersection or overlap occurs so no merge takes places */
  return false;
}

/* find closest position in the routemap */
Position* IsoRoute::ClosestPosition(double lat, double lon, double* dist) {
  double mindist = INFINITY;

  /* first find closest skip position */
  SkipPosition* s = skippoints;
  Position* minpos = s->point;
#if 1
  do {
    Position* p = s->point;

    double dlat = lat - p->lat, dlon = lon - p->lon;
    double dist = dlat * dlat + dlon * dlon;

    if (dist < mindist) {
      minpos = p;
      mindist = dist;
    }

    Position* q = s->next->point;
    switch (s->quadrant) {
      case 0:
        if ((lon > p->lon && lat > p->lat) || (lon < q->lon && lat < q->lat))
          goto skip;
        break;
      case 1:
        if ((lon < p->lon && lat > p->lat) || (lon > q->lon && lat < q->lat))
          goto skip;
        break;
      case 2:
        if ((lat < p->lat && lon > p->lon) || (lat > q->lat && lon < q->lon))
          goto skip;
        break;
      case 3:
        if ((lat < p->lat && lon < p->lon) || (lat > q->lat && lon > q->lon))
          goto skip;
        break;
    }

    {
      Position* e = s->next->point;
      for (p = p->next; p != e; p = p->next) {
        double dlat = lat - p->lat, dlon = lon - p->lon;
        double dist = dlat * dlat + dlon * dlon;

        if (dist < mindist) {
          minpos = p;
          mindist = dist;
        }
      }
    }
  skip:
    s = s->next;
  } while (s != skippoints);

#else
  // this is a lot easier to understand but not as fast as above
  Position* p = s->point;
  do {
    double dlat = lat - p->lat, dlon = lon - p->lon;
    double dist = dlat * dlat + dlon * dlon;

    if (dist < mindist) {
      minpos = p;
      mindist = dist;
    }
    p = p->next;
  } while (p != s->point);
#endif

  /* now try children */
  for (IsoRouteList::iterator it = children.begin(); it != children.end();
       it++) {
    double dist;
    Position* p = (*it)->ClosestPosition(lat, lon, &dist);
    if (/*p &&*/ dist < mindist) {
      minpos = p;
      mindist = dist;
    }
  }

  if (dist) *dist = mindist;

  return minpos;
}

void IsoRoute::ResetDrawnFlag() {
  Position* pos = skippoints->point;
  do {
    pos->drawn = false;
    pos = pos->next;
  } while (pos != skippoints->point);

  for (IsoRouteList::iterator cit = children.begin(); cit != children.end();
       cit++)
    (*cit)->ResetDrawnFlag();
}

bool IsoRoute::Propagate(IsoRouteList& routelist,
                         RouteMapConfiguration& configuration) {
  Position* p = skippoints->point;
  bool ret = false;
  if (p) do {
      if (p->Propagate(routelist, configuration)) ret = true;
      p = p->next;
    } while (p != skippoints->point);
  return ret;
}

void IsoRoute::PropagateToEnd(RouteMapConfiguration& configuration,
                              double& mindt, Position*& endp, double& minH,
                              bool& mintacked, int& mindata_mask) {
  Position* p = skippoints->point;

  do {
    double H;
    int data_mask = 0;
    double dt = p->PropagateToEnd(configuration, H, data_mask);

    /* did we tack thru the wind? apply penalty */
    bool tacked = false;
    if (!std::isnan(dt) && p->parent_heading * H < 0 &&
        fabs(p->parent_heading - H) < 180) {
      tacked = true;
      dt += configuration.TackingTime;
#if 0        
            if(configuration.MaxTacks >= 0 && p->tacks >= configuration.MaxTacks)
                dt = NAN;
#endif
    }

    if (!std::isnan(dt) && dt < mindt) {
      mindt = dt;
      minH = H;
      endp = p;
      mintacked = tacked;
      mindata_mask = data_mask;
    }
    p = p->next;
  } while (p != skippoints->point);

  for (IsoRouteList::iterator cit = children.begin(); cit != children.end();
       cit++)
    (*cit)->PropagateToEnd(configuration, mindt, endp, minH, mintacked,
                           mindata_mask);
}

int IsoRoute::SkipCount() {
  SkipPosition* s = skippoints;
  int count = 0;
  if (s) do {
      count++;
      s = s->next;
    } while (s != skippoints);
  return count;
}

int IsoRoute::Count() {
  Position* p = skippoints->point;
  int count = 0;
  if (p) do {
      count++;
      p = p->next;
    } while (p != skippoints->point);
  return count;
}

void IsoRoute::UpdateStatistics(int& routes, int& invroutes, int& skippositions,
                                int& positions) {
  invroutes += children.size();
  routes += children.size() + 1;

  for (IsoRouteList::iterator it = children.begin(); it != children.end(); it++)
    skippositions += (*it)->SkipCount();
  skippositions += SkipCount();

  for (IsoRouteList::iterator it = children.begin(); it != children.end(); it++)
    positions += (*it)->Count();
  positions += Count();
}

typedef wxWeakRef<Shared_GribRecordSet> Shared_GribRecordSetRef;

static std::map<time_t, Shared_GribRecordSetRef> grib_key;
static wxMutex s_key_mutex;

IsoChron::IsoChron(IsoRouteList r, wxDateTime t, double d,
                   Shared_GribRecordSet& g, bool grib_is_data_deficient)
    : routes(r),
      time(t),
      delta(d),
      m_SharedGrib(g),
      m_Grib(0),
      m_Grib_is_data_deficient(grib_is_data_deficient) {
  m_Grib = m_SharedGrib.GetGribRecordSet();
  if (m_Grib) {
    wxMutexLocker lock(s_key_mutex);
    grib_key[m_Grib->m_Reference_Time] = &m_SharedGrib;
  }
}

Shared_GribRecordSetData::~Shared_GribRecordSetData() {
  delete m_GribRecordSet;
}

IsoChron::~IsoChron() {
  for (IsoRouteList::iterator it = routes.begin(); it != routes.end(); ++it)
    delete *it;
}

void IsoChron::PropagateIntoList(IsoRouteList& routelist,
                                 RouteMapConfiguration& configuration) {
  for (IsoRouteList::iterator it = routes.begin(); it != routes.end(); ++it) {
    bool propagated = false;

    IsoRoute* x = NULL;
    /* if anchoring is allowed, then we can propagate a second time,
       so copy the list before clearing the propagate flag,
       when depth data is implemented we will need to flag positions as
       propagated if they are too deep to anchor here. */
    if (configuration.Anchoring) x = new IsoRoute(*it);

    /* build up a list of iso regions for each point
       in the current iso */
    if ((*it)->Propagate(routelist, configuration)) propagated = true;

    if (!configuration.Anchoring) x = new IsoRoute(*it);

    for (IsoRouteList::iterator cit = (*it)->children.begin();
         cit != (*it)->children.end(); cit++) {
      IsoRoute* y;
      if (configuration.Anchoring)
        y = new IsoRoute(*cit, x);
      else
        y = NULL;
      if ((*cit)->Propagate(routelist, configuration)) {
        if (!configuration.Anchoring) y = new IsoRoute(*cit, x);
        x->children.push_back(y); /* copy child */
        propagated = true;
      } else
        delete y;
    }

    /* if any propagation occured even for children, then we clone this route
       this prevents backtracking, otherwise, we don't need this route
       (it's a dead end) */
    if (propagated)
      routelist.push_front(x);  // slightly faster
    else
      delete x; /* didn't need it */
  }
}

bool IsoChron::Contains(Position& p) {
  for (IsoRouteList::iterator it = routes.begin(); it != routes.end(); ++it)
    switch ((*it)->Contains(p, true)) {
      case -1:  // treat too close to call as not contained
      case 0:
        continue;
      default:
        return true;
    }
  return false;
}

bool IsoChron::Contains(double lat, double lon) {
  Position p(lat, lon);
  return Contains(p);
}

Position* IsoChron::ClosestPosition(double lat, double lon, wxDateTime* t,
                                    double* d) {
  Position* minpos = NULL;
  double mindist = INFINITY;
  wxDateTime mint;
  for (IsoRouteList::iterator it = routes.begin(); it != routes.end(); ++it) {
    double dist;
    Position* pos = (*it)->ClosestPosition(lat, lon, &dist);
    if (pos && dist < mindist) {
      minpos = pos;
      mindist = dist;
      mint = time;
    }
  }
  if (d) *d = mindist;
  if (t) *t = mint;
  return minpos;
}

void IsoChron::ResetDrawnFlag() {
  for (IsoRouteList::iterator it = routes.begin(); it != routes.end(); ++it)
    (*it)->ResetDrawnFlag();
}

bool RouteMapConfiguration::Update() {
  bool havestart = false, haveend = false;
  PlugIn_Waypoint waypoint;

  if (!RouteGUID.IsEmpty()) {
    if (!StartGUID.IsEmpty() && GetSingleWaypoint(StartGUID, &waypoint)) {
      StartLat = waypoint.m_lat;
      StartLon = waypoint.m_lon;
      havestart = true;
    }
    if (!EndGUID.IsEmpty() && GetSingleWaypoint(EndGUID, &waypoint)) {
      EndLat = waypoint.m_lat;
      EndLon = waypoint.m_lon;
      haveend = true;
    }
  } else
    for (const auto& it : RouteMap::Positions) {
      if (Start == it.Name) {
        double lat = it.lat;
        double lon = it.lon;
        if (!it.GUID.IsEmpty() && GetSingleWaypoint(it.GUID, &waypoint)) {
          lat = waypoint.m_lat;
          lon = waypoint.m_lon;
        }
        StartLat = lat;
        StartLon = lon;

        havestart = true;
      }
      if (End == it.Name) {
        double lat = it.lat;
        double lon = it.lon;
        if (!it.GUID.IsEmpty() && GetSingleWaypoint(it.GUID, &waypoint)) {
          lat = waypoint.m_lat;
          lon = waypoint.m_lon;
        }
        EndLat = lat;
        EndLon = lon;
        haveend = true;
      }
    }

  if (!havestart || !haveend) {
    StartLat = StartLon = EndLat = EndLon = NAN;
    return false;
  }

  if ((positive_longitudes = fabs(average_longitude(StartLon, EndLon)) > 90)) {
    StartLon = positive_degrees(StartLon);
    EndLon = positive_degrees(EndLon);
  }

  ll_gc_ll_reverse(StartLat, StartLon, EndLat, EndLon, &StartEndBearing, 0);

  DegreeSteps.clear();
  if (RouteGUID.IsEmpty()) {
    // ensure validity
    FromDegree = wxMax(wxMin(FromDegree, 180), 0);
    ToDegree = wxMax(wxMin(ToDegree, 180), 0);
    if (FromDegree > ToDegree) FromDegree = ToDegree;
    ByDegrees = wxMax(wxMin(ByDegrees, 60), .1);

    for (double step = FromDegree; step <= ToDegree; step += ByDegrees) {
      DegreeSteps.push_back(step);
      if (step > 0 && step < 180) DegreeSteps.push_back(360 - step);
    }
  } else {
    DegreeSteps.push_back(0.);
  }
  DegreeSteps.sort();

  return true;
}

bool (*RouteMap::ClimatologyData)(int setting, const wxDateTime&, double,
                                  double, double&, double&) = NULL;
bool (*RouteMap::ClimatologyWindAtlasData)(const wxDateTime&, double, double,
                                           int& count, double*, double*,
                                           double&, double&) = NULL;
int (*RouteMap::ClimatologyCycloneTrackCrossings)(double, double, double,
                                                  double, const wxDateTime&,
                                                  int) = NULL;

OD_FindClosestBoundaryLineCrossing RouteMap::ODFindClosestBoundaryLineCrossing =
    NULL;

std::list<RouteMapPosition> RouteMap::Positions;

RouteMap::RouteMap() {}

RouteMap::~RouteMap() { Clear(); }

void RouteMap::PositionLatLon(wxString Name, double& lat, double& lon) {
  for (std::list<RouteMapPosition>::iterator it = Positions.begin();
       it != Positions.end(); it++)
    if ((*it).Name == Name) {
      lat = (*it).lat;
      lon = (*it).lon;
    }
}

bool RouteMap::ReduceList(IsoRouteList& merged, IsoRouteList& routelist,
                          RouteMapConfiguration& configuration) {
  IsoRouteList unmerged;
  while (!routelist.empty()) {
    IsoRoute* r1 = routelist.front();
    routelist.pop_front();
    while (!routelist.empty()) {
      if (TestAbort()) return false;

      IsoRoute* r2 = routelist.front();
      routelist.pop_front();
      IsoRouteList rl;

      if (Merge(rl, r1, r2, 0, configuration.InvertedRegions)) {
        routelist.splice(routelist.end(), rl);
        goto remerge;
      } else
        unmerged.push_back(r2);
    }
    /* none more in list so nothing left to merge with */
    merged.push_back(r1);

  remerge:
    /* put any unmerged back in list to continue */
    routelist.splice(routelist.end(), unmerged);
  }
  return true;
}

/* enlarge the map by 1 level */
bool RouteMap::Propagate() {
  Lock();

  if (m_bNeedsGrib) {  // waiting for timer in main thread to request the grib
    Unlock();
    return false;
  }

  if (!m_bValid) { /* config change */
    m_bFinished = true;
    Unlock();
    return false;
  }

  //
  RouteMapConfiguration configuration = m_Configuration;
  configuration.polar_failed = false;
  configuration.wind_data_failed = false;
  configuration.boundary_crossing = false;
  configuration.land_crossing = false;

  // reset grib data deficient flag
  bool grib_is_data_deficient = false;

  if (m_Configuration.AllowDataDeficient &&
      (!m_NewGrib || !m_NewGrib->m_GribRecordPtrArray[Idx_WIND_VX] ||
       !m_NewGrib->m_GribRecordPtrArray[Idx_WIND_VY]) &&
      origin.size() &&
      /*m_Configuration.ClimatologyType <= RouteMapConfiguration::CURRENTS_ONLY
         &&*/
      m_Configuration.UseGrib) {
    SetNewGrib(origin.back()->m_Grib);
    grib_is_data_deficient = true;
  }

  Shared_GribRecordSet shared_grib = m_SharedNewGrib;
  wxDateTime time = m_NewTime;
  double delta;

  m_NewGrib = 0;
  m_SharedNewGrib.SetGribRecordSet(0);

  // request the next grib
  // in a different thread (grib record averaging going in parallel)
  delta = configuration.DeltaTime;
  m_NewTime += wxTimeSpan(0, 0, delta);
  m_bNeedsGrib = configuration.UseGrib;

  Unlock();

  IsoRouteList routelist;
  if (origin.empty()) {
    Position* np = new Position(configuration.StartLat, configuration.StartLon);
    np->prev = np->next = np;
    routelist.push_back(new IsoRoute(np->BuildSkipList()));
    configuration.grib = NULL;
  } else {
    configuration.grib = origin.back()->m_Grib;
    configuration.time = origin.back()->time;
    configuration.UsedDeltaTime = origin.back()->delta;
    configuration.grib_is_data_deficient =
        origin.back()->m_Grib_is_data_deficient;
    // will the grib data work for us?
    if (m_Configuration.UseGrib &&
        (!configuration.grib ||
         !configuration.grib->m_GribRecordPtrArray[Idx_WIND_VX] ||
         !configuration.grib->m_GribRecordPtrArray[Idx_WIND_VY]) &&
        (!RouteMap::ClimatologyData ||
         m_Configuration.ClimatologyType <=
             RouteMapConfiguration::CURRENTS_ONLY)) {
      Lock();
      m_bFinished = true;
      m_bGribFailed = true;
      Unlock();
      return false;
    }

    origin.back()->PropagateIntoList(routelist, configuration);
  }

  IsoChron* update;
  if (routelist.empty()) {
    update = NULL;
  } else {
    IsoRouteList merged;
    if (!ReduceList(merged, routelist, configuration)) return false;

    for (IsoRouteList::iterator it = merged.begin(); it != merged.end(); ++it)
      (*it)->ReduceClosePoints();

    update =
        new IsoChron(merged, time, delta, shared_grib, grib_is_data_deficient);
  }

  Lock();
  if (update) {
    origin.push_back(update);
    if (update->Contains(m_Configuration.EndLat, m_Configuration.EndLon)) {
      SetFinished(true);
    }
  } else
    m_bFinished = true;

  // take note of possible failure reasons
  UpdateStatus(configuration);

  Unlock();

  return true;
}

Position* RouteMap::ClosestPosition(double lat, double lon, wxDateTime* t,
                                    double* d) {
  if (origin.empty()) return NULL;

  Position* minpos = NULL;
  double mindist = INFINITY;
  bool inside;
  bool first = (t != 0);
  wxDateTime min_t;
  Lock();

  IsoChronList::iterator it = origin.end();

  Position p(lat,
             m_Configuration.positive_longitudes ? positive_degrees(lon) : lon);
  do {
    it--;
    double dist;
    wxDateTime cur_t;
    Position* pos = (*it)->ClosestPosition(p.lat, p.lon, &cur_t, &dist);

    if (dist > mindist) break;

    if (pos && dist <= mindist) {
      minpos = pos;
      mindist = dist;
      if (!min_t.IsValid() || (cur_t.IsValid() && cur_t < min_t)) min_t = cur_t;
    }
    /* bail if we don't contain because obviously we aren't getting any closer
     */

    inside = (*it)->Contains(p);
    if (!inside && !first) break;
    if (inside) first = false;
  } while (it != origin.begin());

  Unlock();

  if (d) *d = mindist;
  if (t) *t = min_t;
  return minpos;
}

void RouteMap::Reset() {
  Lock();
  Clear();

  m_NewGrib = NULL;
  m_SharedNewGrib.SetGribRecordSet(0);

  m_NewTime = m_Configuration.StartTime;
  m_bNeedsGrib = m_Configuration.UseGrib && m_Configuration.RouteGUID.IsEmpty();
  m_ErrorMsg = wxEmptyString;

  m_bReachedDestination = false;
  m_bGribFailed = false;
  m_bPolarFailed = false;
  m_bNoData = false;
  m_bFinished = false;
  m_bLandCrossing = false;
  m_bBoundaryCrossing = false;

  Unlock();
}

void RouteMap::SetNewGrib(GribRecordSet* grib) {
  if (!grib || !grib->m_GribRecordPtrArray[Idx_WIND_VX] ||
      !grib->m_GribRecordPtrArray[Idx_WIND_VY])
    return;

  // XXX should be grib->m_ID in a newer OpenCPN version
  unsigned int bogus_ID;  // grib->m_ID

  GribRecord* tmp = grib->m_GribRecordPtrArray[Idx_WIND_VX];
  // RecordRefDate is time_t and high byte is likely the same in many grib
  // files, add some entropy
  bogus_ID = tmp->getRecordRefDate() ^ (tmp->getIdCenter() << 24) ^
             (tmp->getNi() << 16);

  {
    std::map<time_t, Shared_GribRecordSetRef>::iterator it;
    wxMutexLocker lock(s_key_mutex);
    it = grib_key.find(grib->m_Reference_Time);
    if (it != grib_key.end() && it->second != 0) {
      m_SharedNewGrib = *it->second;
      m_NewGrib = m_SharedNewGrib.GetGribRecordSet();
      // compute fake generation grib->m_ID
      if (m_NewGrib->m_ID == bogus_ID) {
        return;
      }
    }
  }
  /* copy the grib record set */
  m_NewGrib = new WR_GribRecordSet(bogus_ID /* XXX */);
  m_NewGrib->m_Reference_Time = grib->m_Reference_Time;
  for (int i = 0; i < Idx_COUNT; i++) {
    switch (i) {
      case Idx_HTSIGW:
      case Idx_WIND_GUST:
      case Idx_WIND_VX:
      case Idx_WIND_VY:
      case Idx_SEACURRENT_VX:
      case Idx_SEACURRENT_VY:
        if (grib->m_GribRecordPtrArray[i]) {
          m_NewGrib->SetUnRefGribRecord(
              i, new GribRecord(*grib->m_GribRecordPtrArray[i]));
        }
        break;
      default:
        break;
    }
  }
  m_SharedNewGrib.SetGribRecordSet(m_NewGrib);
}

void RouteMap::SetNewGrib(WR_GribRecordSet* grib) {
  if (!grib || !grib->m_GribRecordPtrArray[Idx_WIND_VX] ||
      !grib->m_GribRecordPtrArray[Idx_WIND_VY])
    return;

  {
    std::map<time_t, Shared_GribRecordSetRef>::iterator it;
    wxMutexLocker lock(s_key_mutex);
    it = grib_key.find(grib->m_Reference_Time);
    if (it != grib_key.end() && it->second != 0) {
      m_SharedNewGrib = *it->second;
      m_NewGrib = m_SharedNewGrib.GetGribRecordSet();
      if (m_NewGrib->m_ID == grib->m_ID) {
        return;
      }
    }
  }
  /* copy the grib record set */
  m_NewGrib = new WR_GribRecordSet(grib->m_ID);
  m_NewGrib->m_Reference_Time = grib->m_Reference_Time;
  for (int i = 0; i < Idx_COUNT; i++) {
    switch (i) {
      case Idx_HTSIGW:
      case Idx_WIND_GUST:
      case Idx_WIND_VX:
      case Idx_WIND_VY:
      case Idx_SEACURRENT_VX:
      case Idx_SEACURRENT_VY:
        if (grib->m_GribRecordPtrArray[i]) {
          m_NewGrib->SetUnRefGribRecord(
              i, new GribRecord(*grib->m_GribRecordPtrArray[i]));
        }
        break;
      default:
        break;
    }
  }
  m_SharedNewGrib.SetGribRecordSet(m_NewGrib);
}

void RouteMap::GetStatistics(int& isochrons, int& routes, int& invroutes,
                             int& skippositions, int& positions) {
  Lock();
  isochrons = origin.size();
  routes = invroutes = skippositions = positions = 0;
  for (IsoChronList::iterator it = origin.begin(); it != origin.end(); ++it)
    for (IsoRouteList::iterator rit = (*it)->routes.begin();
         rit != (*it)->routes.end(); ++rit)
      (*rit)->UpdateStatistics(routes, invroutes, skippositions, positions);
  Unlock();
}

void RouteMap::Clear() {
  for (IsoChronList::iterator it = origin.begin(); it != origin.end(); ++it)
    delete *it;

  origin.clear();
}
