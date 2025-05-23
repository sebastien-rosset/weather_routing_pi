/***************************************************************************
 *   Copyright (C) 2024 by OpenCPN development team                        *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
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
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 **************************************************************************/

#ifdef __WINDOWS__
#include <windows.h>
#endif

#include <wx/wx.h>
#include <wx/aui/framemanager.h>
#include <wx/bitmap.h>
#include <wx/fileconf.h>
#include <wx/font.h>
#include <wx/string.h>
#include <wx/window.h>
#include <wx/event.h>

#include <vector>
#include <memory>
#include <string>

#include "ocpn_plugin.h"
#include "mock_plugin_api.h"

// Global counter to track NMEA sentences pushed to buffer
static std::vector<wxString> g_nmea_sentences;

// Helper functions to access mock state
void ClearNMEASentences() { g_nmea_sentences.clear(); }

const std::vector<wxString> &GetNMEASentences() { return g_nmea_sentences; }

// Plugin API mock implementations
extern "C" {

DECL_EXP int GetChartbarHeight(void) { return 1; }
void SendPluginMessage(wxString message_id, wxString message_body) {}
bool AddLocaleCatalog(wxString catalog) { return true; }
bool GetGlobalColor(wxString colorName, wxColour *pcolour) { return true; }
wxFileConfig *GetOCPNConfigObject(void) { return 0; }
wxAuiManager *GetFrameAuiManager(void) { return 0; }
wxWindow *GetOCPNCanvasWindow() { return 0; }

DECL_EXP void PushNMEABuffer(wxString str) {
  g_nmea_sentences.push_back(str.Strip(wxString::both));
}

}  // extern "C"

void RemovePlugInTool(int tool_id) {}
DECL_EXP int InsertPlugInToolSVG(wxString label, wxString SVGfile,
                                 wxString SVGfileRollover,
                                 wxString SVGfileToggled, wxItemKind kind,
                                 wxString shortHelp, wxString longHelp,
                                 wxObject *clientData, int position,
                                 int tool_sel, opencpn_plugin *pplugin) {
  return 0;
}

DECL_EXP int InsertPlugInTool(wxString label, wxBitmap *bitmap,
                              wxBitmap *bmpRollover, wxItemKind kind,
                              wxString shortHelp, wxString longHelp,
                              wxObject *clientData, int position, int tool_sel,
                              opencpn_plugin *pplugin) {
  return 0;
}
void SetToolbarItemState(int item, bool toggle) {}

DECL_EXP int PlatformDirSelectorDialog(wxWindow *parent, wxString *file_spec,
                                       wxString Title, wxString initDir) {
  return 0;
}

DECL_EXP int PlatformFileSelectorDialog(wxWindow *parent, wxString *file_spec,
                                        wxString Title, wxString initDir,
                                        wxString suggestedName,
                                        wxString wildcard) {
  return 0;
}

DECL_EXP wxFont *GetOCPNScaledFont_PlugIn(wxString TextElement,
                                          int default_size) {
  return 0;
}

DECL_EXP wxFont *FindOrCreateFont_PlugIn(int point_size, wxFontFamily family,
                                         wxFontStyle style, wxFontWeight weight,
                                         bool ul, const wxString &face,
                                         wxFontEncoding enc) {
  return 0;
}

// Plugin API mock implementations

wxString *GetpPrivateApplicationDataLocation(void) { return nullptr; }

class ObservableListener {
public:
  ObservableListener(int /* unused */, wxEvtHandler *handler, wxEventType type)
      : m_handler(handler), m_type(type) {}
  wxEvtHandler *m_handler;
  wxEventType m_type;
};

// C++ functions with DECL_EXP go outside extern "C"
std::shared_ptr<ObservableListener> DECL_EXP GetListener(NMEA2000Id id,
                                                         wxEventType et,
                                                         wxEvtHandler *eh) {
  return std::make_shared<ObservableListener>(id.id, eh, et);
}

std::string DECL_EXP GetN2000Source(NMEA2000Id /* id */,
                                    ObservedEvt /* evt */) {
  return std::string("MockSource");
}

std::vector<uint8_t> DECL_EXP GetN2000Payload(NMEA2000Id /* id */,
                                              ObservedEvt /* evt */) {
  return std::vector<uint8_t>{0, 1, 2, 3};  // Mock data
}

wxString DECL_EXP GetPluginDataDir(const char *plugin_name) {
  return wxString("/mock/plugin/data");
}

// Define the wxAuiManager methods
bool wxAuiManager::DetachPane(wxWindow *window) {
  return true;  // Mock implementation always succeeds
}

bool wxAuiManager::AddPane(wxWindow *window, const wxAuiPaneInfo &pane_info) {
  return true;  // Mock implementation always succeeds
}

void wxAuiManager::Update() {
  // Mock implementation does nothing
}

wxAuiPaneInfo &wxAuiManager::GetPane(wxWindow *window) {
  static wxAuiPaneInfo info;
  return info;  // Return a static instance for mocking
}

bool wxAuiPaneInfo::IsValid() const { return true; }

DECL_EXP bool PlugIn_GSHHS_CrossesLand(double lat1, double lon1, double lat2,
                                       double lon2) {
  return true;
}
DECL_EXP bool GetSingleWaypoint(wxString GUID, PlugIn_Waypoint *pwaypoint) {
  return true;
}

DECL_EXP PlugIn_Waypoint::PlugIn_Waypoint() {}
DECL_EXP PlugIn_Waypoint::PlugIn_Waypoint(double, double, const wxString &,
                                          const wxString &, const wxString &) {}
DECL_EXP PlugIn_Waypoint::~PlugIn_Waypoint() {}

DECL_EXP PlugIn_Waypoint_Ex::PlugIn_Waypoint_Ex() {}
DECL_EXP PlugIn_Waypoint_Ex::PlugIn_Waypoint_Ex(
    double lat, double lon, const wxString &icon_ident, const wxString &wp_name,
    const wxString &GUID, const double ScaMin, const bool bNameVisible,
    const int nRanges, const double RangeDistance, const wxColor RangeColor) {}
DECL_EXP PlugIn_Waypoint_Ex::~PlugIn_Waypoint_Ex() {}
DECL_EXP void PlugIn_Waypoint_Ex::InitDefaults() {}

DECL_EXP bool PlugIn_Waypoint_Ex::GetFSStatus() { return true; }

DECL_EXP int PlugIn_Waypoint_Ex::GetRouteMembershipCount() { return 0; }
DECL_EXP std::unique_ptr<PlugIn_Waypoint> GetWaypoint_Plugin(const wxString &) {
  return nullptr;
}
DECL_EXP std::unique_ptr<PlugIn_Route> GetRoute_Plugin(const wxString &) {
  return nullptr;
}
DECL_EXP std::unique_ptr<PlugIn_Track> GetTrack_Plugin(const wxString &) {
  return nullptr;
}
DECL_EXP bool AddPlugInRoute(PlugIn_Route *proute, bool b_permanent) {
  return true;
}
DECL_EXP bool DeletePlugInRoute(wxString &GUID) { return true; }
DECL_EXP bool UpdatePlugInRoute(PlugIn_Route *proute) { return true; }

DECL_EXP bool AddPlugInRouteEx(PlugIn_Route_Ex *proute, bool b_permanent) {
  return true;
}
DECL_EXP bool UpdatePlugInRouteEx(PlugIn_Route_Ex *proute) { return true; }
DECL_EXP bool AddPlugInTrack(PlugIn_Track *ptrack, bool b_permanent) {
  return true;
}
DECL_EXP PlugIn_Route::PlugIn_Route(void) {}
DECL_EXP PlugIn_Route::~PlugIn_Route(void) {}

DECL_EXP PlugIn_Route_Ex::PlugIn_Route_Ex(void) {}
DECL_EXP PlugIn_Route_Ex::~PlugIn_Route_Ex(void) {}

DECL_EXP PlugIn_Track::PlugIn_Track() {}
DECL_EXP PlugIn_Track::~PlugIn_Track() {}

DECL_EXP wxString GetSelectedWaypointGUID_Plugin() { return wxString(""); }
DECL_EXP wxString GetSelectedRouteGUID_Plugin() { return wxString(""); }
DECL_EXP wxString GetSelectedTrackGUID_Plugin() { return wxString(""); }

DECL_EXP int AddCanvasContextMenuItem(wxMenuItem *pitem,
                                      opencpn_plugin *pplugin) {
  return 1;
}
DECL_EXP int AddCanvasMenuItem(wxMenuItem *pitem, opencpn_plugin *pplugin,
                               const char *name) {
  return 0;
}
void DimeWindow(wxWindow *win) {}
DECL_EXP void RequestRefresh(wxWindow *) {}
DECL_EXP void SetCanvasMenuItemViz(int item, bool viz, const char *name) {}
DECL_EXP wxString GetLocaleCanonicalName() { return wxString(""); }
DECL_EXP double DistGreatCircle_Plugin(double slat, double slon, double dlat,
                                       double dlon) {
  return 0.0;
}
DECL_EXP void DistanceBearingMercator_Plugin(double lat0, double lon0,
                                             double lat1, double lon1,
                                             double *brg, double *dist) {}
DECL_EXP void JumpToPosition(double lat, double lon, double scale) {};

DECL_EXP double toUsrDistance_Plugin(double nm_distance, int unit) {
  return 0.0;
}
DECL_EXP double fromUsrDistance_Plugin(double usr_distance, int unit) {
  return 0.0;
}
DECL_EXP double toUsrSpeed_Plugin(double kts_speed, int unit) { return 0.0; }
DECL_EXP double fromUsrSpeed_Plugin(double usr_speed, int unit) { return 0.0; }
DECL_EXP double toUsrTemp_Plugin(double cel_temp, int unit) { return 0.0; }
DECL_EXP double fromUsrTemp_Plugin(double usr_temp, int unit) { return 0.0; }
DECL_EXP wxString getUsrDistanceUnit_Plugin(int unit) { return ""; }
DECL_EXP wxString getUsrSpeedUnit_Plugin(int unit) { return ""; }
DECL_EXP wxString getUsrTempUnit_Plugin(int unit) { return ""; }

DECL_EXP wxString GetNewGUID() { return ""; }
DECL_EXP wxString toSDMM_PlugIn(int NEflag, double a, bool hi_precision) {
  return "";
}
DECL_EXP int GetLatLonFormat(void) { return 0; }
DECL_EXP void GetCanvasPixLL(PlugIn_ViewPort *vp, wxPoint *pp, double lat,
                             double lon) {}
DECL_EXP void GetCanvasLLPix(PlugIn_ViewPort *vp, wxPoint p, double *plat,
                             double *plon) {}
DECL_EXP void GetDoubleCanvasPixLL(PlugIn_ViewPort *vp, wxPoint2DDouble *pp,
                                   double lat, double lon) {}
