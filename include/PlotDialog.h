/***************************************************************************
 *
 * Project:  OpenCPN Weather Routing plugin
 * Author:   Sean D'Epagnier
 *
 ***************************************************************************
 *   Copyright (C) 2015 by Sean D'Epagnier                                 *
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
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301,  USA.             *
 ***************************************************************************
 */

#ifndef _PLOT_DIALOG_H_
#define _PLOT_DIALOG_H_

#include <list>

#include <wx/fileconf.h>

#include "WeatherRoutingUI.h"

class weather_routing_pi;
class WeatherRouting;

class PlotDialog : public PlotDialogBase {
public:
  PlotDialog(WeatherRouting& weatherrouting);
  ~PlotDialog();

  void SetRouteMapOverlay(RouteMapOverlay* routemapoverlay);

private:
  /**
   * Navigation and weather variables used in route calculations.
   *
   * This enum represents various navigation parameters related to vessel
   * movement, wind conditions, and sea state that are used in weather routing
   * calculations.
   */
  enum Variable {
    /** Vessel's speed relative to the earth's surface in knots (SOG) */
    SPEED_OVER_GROUND,
    /** Vessel's direction of travel relative to the earth's surface in degrees
       (COG) */
    COURSE_OVER_GROUND,
    /** Vessel's speed relative to the water in knots (STW) */
    SPEED_OVER_WATER,
    /** Vessel's direction of travel relative to the water in degrees (CTW) -
       differs from heading by accounting for leeway */
    COURSE_OVER_WATER,
    /** Wind speed relative to the water's frame of reference in knots. */
    WIND_VELOCITY,
    /** Relative angle between vessel's course through water and the wind
       direction in degrees. */
    WIND_DIRECTION,
    /** Absolute wind direction over water in degrees (meteorological, where
       wind is coming FROM) */
    WIND_COURSE,
    /** True wind speed relative to the earth's surface in knots (TWS) */
    WIND_VELOCITY_GROUND,
    /** True Wind direction relative to the vessel's course over ground in degrees
       (relative angle) */
    WIND_DIRECTION_GROUND,
    /** Absolute true wind direction in degrees relative to true north (TWD) -
       meteorological, where wind is coming FROM */
    WIND_COURSE_GROUND,
    /** Wind speed as experienced by the vessel in knots (AWS) */
    APPARENT_WIND_SPEED,
    /** Angle between vessel heading and apparent wind in degrees (AWA) */
    APPARENT_WIND_ANGLE,
    /** Maximum wind speed in gusts in knots */
    WIND_GUST,
    /** Water current speed in knots */
    CURRENT_VELOCITY,
    /** Water current direction in degrees (from direction) */
    CURRENT_DIRECTION,
    /** Significant wave height in meters */
    SIG_WAVE_HEIGHT,
    /** Number of tacking maneuvers in a sailing route */
    TACKS
  };

  void OnMouseEventsPlot(wxMouseEvent& event);
  void OnPaintPlot(wxPaintEvent& event);
  void OnSizePlot(wxSizeEvent& event) { m_PlotWindow->Refresh(); }
  void OnUpdatePlot(wxScrollEvent& event) { m_PlotWindow->Refresh(); }
  void OnUpdatePlotVariable(wxCommandEvent& event) {
    GetScale();
    m_PlotWindow->Refresh();
  }
  void OnUpdateRoute(wxCommandEvent& event);
  void OnUpdateUI(wxUpdateUIEvent& event);

private:
  double GetValue(PlotData& data, int var);
  int GetType(int var);
  void GetScale();

  wxDateTime m_StartTime;

  double m_mintime, m_maxtime;
  double m_minvalue[3], m_maxvalue[3];

  std::list<PlotData> m_PlotData;

  WeatherRouting& m_WeatherRouting;
};

#endif
