/**************************************************************************
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
 ***************************************************************************/

#ifndef _WEATHER_ROUTING_H_
#define _WEATHER_ROUTING_H_

#include <wx/treectrl.h>
#include <wx/fileconf.h>
#include <wx/collpane.h>

#ifdef __OCPN__ANDROID__
#include <wx/qt/private/wxQtGesture.h>
#endif
#include "ocpn_plugin.h"

#include "WeatherRoutingUI.h"
#include "ConfigurationDialog.h"
#include "ConfigurationBatchDialog.h"
#include "BoatDialog.h"
#include "SettingsDialog.h"
#include "StatisticsDialog.h"
#include "ReportDialog.h"
#include "PlotDialog.h"
#include "FilterRoutesDialog.h"

class weather_routing_pi;
class WeatherRouting;

/**
 * Class representing a weather routing configuration and its associated route.
 *
 * This class serves as a UI-focused wrapper that combines both configuration
 * settings and calculation results. While RouteMapConfiguration stores the raw
 * routing parameters and RouteMapOverlay handles the actual route computation
 * and display, WeatherRoute maintains the human-readable representation of
 * route data for display in the UI.
 *
 * The class stores formatted strings for distance, speed, weather conditions,
 * and other metrics rather than raw numerical values. It works closely with the
 * WeatherRouting dialog to present route information in a user-friendly format.
 *
 * @see RouteMapConfiguration For the underlying route parameters and settings
 * @see RouteMapOverlay For the actual route calculation and display
 * functionality
 * @see WeatherRouting For the main routing interface that manages these routes
 */
class WeatherRoute {
public:
  WeatherRoute();
  ~WeatherRoute();

  /**
   * Updates the weather route object with current configuration and calculation
   * status.
   *
   * This method updates both the route configuration data and status
   * information. It's used whenever route information needs to be refreshed in
   * the UI.
   *
   * @param wr Pointer to the WeatherRouting main object to access settings.
   * @param stateonly If true, only update the State field, not configuration
   * data.
   */
  void Update(WeatherRouting* wr, bool stateonly = false);

  /** Flag indicating if this route is filtered out in the UI display. */
  bool Filtered;

  /** Path to the boat characteristics file used for this route. */
  wxString BoatFilename;

  /** Starting position name/coordinates. */
  wxString Start;

  /** Specifies whether to set the StartTime to the current computer time at the
   * start of the calculation. */
  wxString UseCurrentTime;

  /** Starting position type (boat or named). */
  wxString StartType;

  /** Departure time for the route. */
  wxString StartTime;

  /** Destination position name/coordinates. */
  wxString End;

  /** Estimated arrival time at destination. */
  wxString EndTime;

  /** Total route duration. */
  wxString Time;

  /** Total route distance in nautical miles. */
  wxString Distance;

  /** Average boat speed through water in knots. */
  wxString AvgSpeed;

  /** Maximum boat speed through water in knots. */
  wxString MaxSpeed;

  /** Average boat speed over ground including currents in knots. */
  wxString AvgSpeedGround;

  /** Maximum boat speed over ground in knots. */
  wxString MaxSpeedGround;

  /** Average wind speed encountered in knots. */
  wxString AvgWind;

  /** Maximum sustained wind speed encountered in knots. */
  wxString MaxWind;

  /** Maximum wind gust encountered in knots. */
  wxString MaxWindGust;

  /** Average current speed encountered in knots. */
  wxString AvgCurrent;

  /** Maximum current speed encountered in knots. */
  wxString MaxCurrent;

  /** Average swell height encountered in meters. */
  wxString AvgSwell;

  /** Maximum swell height encountered in meters. */
  wxString MaxSwell;

  /** Percentage of time spent sailing upwind. */
  wxString UpwindPercentage;

  /** Distribution between port and starboard tacks. */
  wxString PortStarboard;

  /** Number of tacks/gybes performed. */
  wxString Tacks;

  /** Current computation state of the route. */
  wxString State;

  /** Comfort/safety metrics for the route conditions. */
  wxString Comfort;

  /** Pointer to the actual route calculation and display overlay. */
  RouteMapOverlay* routemapoverlay;
};

class WeatherRouting : public WeatherRoutingBase {
private:
  bool m_disable_colpane;
  wxCollapsiblePane* m_colpane;
  wxWindow* m_colpaneWindow;
  WeatherRoutingPanel* m_panel;

public:
  enum {
    POSITION_NAME = 0,  //!< Position identifier/name
    POSITION_LAT,       //!< Latitude coordinate
    POSITION_LON        //!< Longitude coordinate
  };

  enum {
    VISIBLE = 0,        //!< Route visibility toggle state
    BOAT,               //!< Boat configuration file name
    START,              //!< Starting position name/coordinates
    STARTTIME,          //!< Route departure time
    END,                //!< Destination position name/coordinates
    ENDTIME,            //!< Estimated arrival time
    TIME,               //!< Total route duration
    DISTANCE,           //!< Total route distance in nautical miles
    AVGSPEED,           //!< Average boat speed through water in knots
    MAXSPEED,           //!< Maximum boat speed through water in knots
    AVGSPEEDGROUND,     //!< Average boat speed over ground in knots
    MAXSPEEDGROUND,     //!< Maximum boat speed over ground in knots
    AVGWIND,            //!< Average wind speed encountered in knots
    MAXWIND,            //!< Maximum sustained wind speed in knots
    MAXWINDGUST,        //!< Maximum wind gust encountered in knots
    AVGCURRENT,         //!< Average current speed encountered in knots
    MAXCURRENT,         //!< Maximum current speed encountered in knots
    AVGSWELL,           //!< Average swell height encountered in meters
    MAXSWELL,           //!< Maximum swell height encountered in meters
    UPWIND_PERCENTAGE,  //!< Percentage of time spent sailing upwind
    PORT_STARBOARD,     //!< Distribution between port and starboard tacks
    TACKS,              //!< Number of tacks/gybes performed
    COMFORT,            //!< Comfort/safety metrics for conditions
    STATE,              //!< Current computation state of route
    NUM_COLS            //!< Total number of display columns
  };
  long columns[NUM_COLS];
  static const wxString column_names[NUM_COLS];
  int sashpos;

  WeatherRouting(wxWindow* parent, weather_routing_pi& plugin);
  ~WeatherRouting();

#ifdef __OCPN__ANDROID__
  void OnEvtPanGesture(wxQT_PanGestureEvent& event);
#endif
  void OnLeftDown(wxMouseEvent& event);
  void OnLeftUp(wxMouseEvent& event);
  void OnDownTimer(wxTimerEvent&);

  void Reset();

  void Render(piDC& dc, PlugIn_ViewPort& vp);
  ConfigurationDialog m_ConfigurationDialog;
  ConfigurationBatchDialog m_ConfigurationBatchDialog;
  CursorPositionDialog m_CursorPositionDialog;
  RoutePositionDialog m_RoutePositionDialog;
  BoatDialog m_BoatDialog;

  void SetConfigurationRoute(WeatherRoute* weatherroute);
  void UpdateBoatFilename(wxString boatFileName);

  void UpdateCurrentConfigurations();
  void UpdateStates();
  /**
   * Get list of currently selected route maps in the weather routes list
   *
   * @param messagedialog If true, show warning dialog when no routes are
   * selected
   * @return List of RouteMapOverlay pointers for selected routes
   */
  std::list<RouteMapOverlay*> CurrentRouteMaps(bool messagedialog = false);
  RouteMapOverlay* FirstCurrentRouteMap();
  RouteMapOverlay* m_RouteMapOverlayNeedingGrib;

  void RebuildList();
  std::list<RouteMapOverlay*> m_RunningRouteMaps, m_WaitingRouteMaps;
  std::list<WeatherRoute*> m_WeatherRoutes;

  void GenerateBatch();
  bool Show(bool show);

  void UpdateDisplaySettings();

  void AddPosition(double lat, double lon);
  void AddPosition(double lat, double lon, wxString name);
  void AddPosition(double lat, double lon, wxString name, wxString GUID);
  void AddRoute(wxString& GUID);

  void CursorRouteChanged();
  void UpdateColumns();

  void UpdateCursorPositionDialog();
  void UpdateRoutePositionDialog();

  SettingsDialog m_SettingsDialog;

private:
  void CopyDataFiles(wxString from, wxString to);
  void OnCollPaneChanged(wxCollapsiblePaneEvent& event);
  void OnNewPosition(wxCommandEvent& event);
  void OnUpdateBoat(wxCommandEvent& event);
  void OnDeletePosition(wxCommandEvent& event);
  void OnDeleteAllPositions(wxCommandEvent& event);
  void OnClose(wxCloseEvent& event) { Hide(); }
  void OnPositionKeyDown(wxListEvent& event);
  void OnEditConfiguration();
  void OnOpen(wxCommandEvent& event);
  void OnSave(wxCommandEvent& event);
  void OnClose(wxCommandEvent& event);
  void OnSize(wxSizeEvent& event);
  void OnNew(wxCommandEvent& event);
  void OnEditConfigurationClick(wxMouseEvent& event) { OnEditConfiguration(); }
  void OnWeatherRouteSort(wxListEvent& event);
  void OnWeatherRouteSelected();
  void OnWeatherRouteSelected(wxListEvent& event) { OnWeatherRouteSelected(); }
  void OnWeatherRouteKeyDown(wxListEvent& event);
  void OnWeatherRoutesListLeftDown(wxMouseEvent& event);
  void UpdateComputeState();
  void OnCompute(wxCommandEvent& event);
  void OnComputeAll(wxCommandEvent& event);
  void OnStop(wxCommandEvent& event);
  void OnResetAll(wxCommandEvent& event);
  void OnPositions(wxCommandEvent& event);
  void OnBatch(wxCommandEvent& event);
  void OnEditConfiguration(wxCommandEvent& event) { OnEditConfiguration(); }
  void OnGoTo(wxCommandEvent& event);
  void OnDelete(wxCommandEvent& event);
  void OnDeleteAll(wxCommandEvent& event);
  void OnFilter(wxCommandEvent& event);
  void OnExport(wxCommandEvent& event);
  void OnExportRoute(wxCommandEvent& event);
  void OnExportAll(wxCommandEvent& event);
  void OnSettings(wxCommandEvent& event);
  void OnStatistics(wxCommandEvent& event);
  void OnReport(wxCommandEvent& event);
  void OnPlot(wxCommandEvent& event);
  void OnCursorPosition(wxCommandEvent& event);
  // CUSTOMIZATION
  void OnRoutePosition(wxCommandEvent& event);
  void OnManual(wxCommandEvent& event);
  void OnInformation(wxCommandEvent& event);
  void OnAbout(wxCommandEvent& event);

  void OnComputationTimer(wxTimerEvent&);
  void OnHideConfigurationTimer(wxTimerEvent&);
  void OnRenderedTimer(wxTimerEvent&);

  bool OpenXML(wxString filename, bool reportfailure = true);
  void SaveXML(wxString filename);

  void SetEnableConfigurationMenu();

  void UpdateConfigurations();
  void UpdateDialogs();

  bool AddConfiguration(RouteMapConfiguration& configuration);
  void UpdateRouteMap(RouteMapOverlay* routemapoverlay);
  void UpdateItem(long index, bool stateonly = false);

  RouteMap* SelectedRouteMap();
  void Export(RouteMapOverlay& routemapoverlay);
  void ExportRoute(RouteMapOverlay& routemapoverlay);
  /**
   * Initiates route calculation for a specific route map overlay.
   *
   * This method handles the pre-computation setup for a route map overlay:
   * - If starting from boat position, it updates the start coordinates
   * - It attempts to start the computation thread
   * - It handles any errors that occur during startup
   * - It adds successful starts to the running routes list
   *
   * @param routemapoverlay Pointer to the route map overlay to compute
   */
  void Start(RouteMapOverlay* routemapoverlay);
  void StartAll();
  /* Stop the computation of the specified route. */
  void Stop(RouteMapOverlay* routemapoverlay);
  /* Stop the computation of all routes. */
  void StopAll();

  void DeleteRouteMaps(std::list<RouteMapOverlay*> routemapoverlays);
  RouteMapConfiguration DefaultConfiguration();

  StatisticsDialog m_StatisticsDialog;
  ReportDialog m_ReportDialog;
  PlotDialog m_PlotDialog;
  FilterRoutesDialog m_FilterRoutesDialog;

  wxTimer m_tCompute, m_tHideConfiguration;

  bool m_bRunning;
  wxTimeSpan m_RunTime;
  wxDateTime m_StartTime;

  wxString m_default_configuration_path;

  int m_RoutesToRun;
  bool m_bSkipUpdateCurrentItems;

  bool m_bShowConfiguration, m_bShowConfigurationBatch, m_bShowRoutePosition;
  bool m_bShowSettings, m_bShowStatistics, m_bShowReport, m_bShowPlot,
      m_bShowFilter;

  wxPoint m_downPos, m_startPos, m_startMouse;
  wxTimer m_tDownTimer;

  weather_routing_pi& m_weather_routing_pi;

  wxFileName m_FileName;

  wxSize m_size;

  // CUSTOMIZATION
  RoutePoint* m_positionOnRoute;
  RoutePoint m_savedPosition;
};

#endif
