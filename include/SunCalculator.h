/***************************************************************************
 *   Copyright (C) 2016 by OpenCPN development team                        *
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

#ifndef _WEATHER_ROUTING_SUN_CALCULATOR_H_
#define _WEATHER_ROUTING_SUN_CALCULATOR_H_

#include <wx/wx.h>

#include <vector>
#include <algorithm>

enum class DayLightStatus { Day, Night };

class SunCalculator {
public:
  // Singleton accessor with guaranteed thread safety.
  static SunCalculator& GetInstance() {
    static SunCalculator instance;
    return instance;
  }

  /**
   * Calculates sunrise and sunset times for a specific location and date
   *
   * This function implements the US Naval Observatory algorithm from the
   * "Almanac for Computers, 1990" to compute sunrise and sunset times.
   * The calculated times are returned in UTC.
   *
   * For locations in polar regions, where the sun may not rise or set on
   * certain dates, the function sets the year value of the returned DateTime to
   * 999 to indicate this condition.
   *
   * @param latit Latitude in decimal degrees (positive for North, negative for
   * South)
   * @param longit Longitude in decimal degrees (positive for East, negative for
   * West)
   * @param dayOfYear Reference day of year (in UTC) for which to calculate sun
   * times
   * @param sunrise [out] Returns the sunrise time in UTC for the specified date
   * @param sunset [out] Returns the sunset time in UTC for the specified date
   */
  static void CalculateSun(double latit, double longit, int dayOfYear,
                           wxDateTime& sunrise, wxDateTime& sunset);

  /**
   * Determines whether it's day or night at a specific location and time
   *
   * This function checks if a given time at a particular location is during
   * daylight or nighttime by calculating the sunrise and sunset times. It uses
   * an internal caching mechanism to improve performance for repeated lookups
   * of similar coordinates and dates.
   *
   * The function accounts for special cases in polar regions where the sun may
   * not rise (polar night) or set (midnight sun) on certain dates.
   *
   * @param lat Latitude in decimal degrees (positive for North, negative for
   * South)
   * @param lon Longitude in decimal degrees (positive for East, negative for
   * West)
   * @param time UTC time for which to determine day/night status
   * @return DayTime enum value (Day or Night) indicating whether it's daytime
   * or nighttime
   */
  DayLightStatus GetDayLightStatus(double lat, double lon,
                                   const wxDateTime& time);

private:
  SunCalculator() = default;

  struct SunTimeCache {
    int day_of_year;
    /** The latitude with 1-degree precision. */
    int lat_index;
    /** The longitude with 1-degree precision. */
    int lon_index;
    /** Sunrise time in UTC. */
    wxDateTime sunrise;
    /** Sunset time in UTC. */
    wxDateTime sunset;
    wxDateTime last_accessed;
  };

  static const int MAX_SUN_CACHE_SIZE = 100;
  std::vector<SunTimeCache> m_cache;
  wxCriticalSection m_cacheLock;
};

#endif
