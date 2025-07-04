/***************************************************************************
 *   Copyright (C) 2018 by Sean D'Epagnier                                 *
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
#include <wx/glcanvas.h>

#include <functional>
#include <list>

#include "ocpn_plugin.h"
#include "pidc.h"
#include "json/json.h"
#include "Utilities.h"
#include "Boat.h"
#include "RouteMapOverlay.h"
#include "SettingsDialog.h"
#include "georef.h"

void WR_GetCanvasPixLL(PlugIn_ViewPort* vp, wxPoint* pp, double lat,
                       double lon) {
  wxPoint2DDouble pix_double;
  GetDoubleCanvasPixLL(vp, &pix_double, lat, lon);
  pp->x = (int)wxRound(pix_double.m_x);
  pp->y = (int)wxRound(pix_double.m_y);
}

RouteMapOverlayThread::RouteMapOverlayThread(RouteMapOverlay& routemapoverlay)
    : wxThread(wxTHREAD_JOINABLE), m_RouteMapOverlay(routemapoverlay) {
  Create();
}

void* RouteMapOverlayThread::Entry() {
  RouteMapConfiguration cf = m_RouteMapOverlay.GetConfiguration();

  if (!cf.RouteGUID.IsEmpty()) {
    std::unique_ptr<PlugIn_Route> rte = GetRoute_Plugin(cf.RouteGUID);
    PlugIn_Route* proute = rte.get();
    if (proute == nullptr) return 0;

    m_RouteMapOverlay.RouteAnalysis(proute);
  } else {
    while (!TestDestroy() && !m_RouteMapOverlay.Finished()) {
      if (!m_RouteMapOverlay.Propagate())
        wxThread::Sleep(50);
      else {
        // don't do it inside worker thread, race
        // m_RouteMapOverlay.UpdateCursorPosition();
        m_RouteMapOverlay.UpdateDestination();
        wxThread::Sleep(5);
      }
    }
  }
  //    m_RouteMapOverlay.m_Thread = nullptr;
  return 0;
}

RouteMapOverlay::RouteMapOverlay()
    : m_UpdateOverlay(true),
      m_bEndRouteVisible(false),
      m_Thread(nullptr),
      last_cursor_lat(0),
      last_cursor_lon(0),
      last_cursor_position(nullptr),
      destination_position(nullptr),
      last_destination_position(nullptr),
      m_bUpdated(false),
      m_overlaylist(0),
      clear_destination_plotdata(false),
      wind_barb_cache_scale(NAN),
      wind_barb_cache_origin_size(0),
      current_cache_scale(NAN),
      current_cache_origin_size(0) {}

RouteMapOverlay::~RouteMapOverlay() {
  delete destination_position;

  if (m_Thread) Stop();
}

bool RouteMapOverlay::Start(wxString& error) {
  if (m_Thread) {
    error = _("error, thread already created\n");
    return false;
  }

  error = LoadBoat();
  if (error.size()) return false;

  RouteMapConfiguration configuration = GetConfiguration();
  /* test for cyclone data if needed */
  if (configuration.AvoidCycloneTracks &&
      (!ClimatologyCycloneTrackCrossings ||
       ClimatologyCycloneTrackCrossings(0, 0, 0, 0, wxDateTime(), 0) == -1)) {
    error =
        _("Configuration specifies cyclone track avoidance and Climatology "
          "cyclone data is not available");
    return false;
  }

  if (configuration.DetectBoundary &&
      !RouteMap::ODFindClosestBoundaryLineCrossing) {
    error =
        _("Configuration specifies boundary exclusion but ocpn_draw_pi "
          "boundary data not available");
    return false;
  }

  if (!configuration.UseGrib &&
      configuration.ClimatologyType <= RouteMapConfiguration::CURRENTS_ONLY) {
    error = _("Configuration does not allow grib or climatology wind data");
    return false;
  }

  m_Thread = new RouteMapOverlayThread(*this);
  m_Thread->Run();
  return true;
}

void RouteMapOverlay::RouteAnalysis(PlugIn_Route* proute) {
  std::list<PlotData>& plotdata = last_destination_plotdata;
  RouteMapConfiguration configuration = GetConfiguration();

  configuration.polar_status = POLAR_SPEED_SUCCESS;
  configuration.wind_data_status = wxEmptyString;
  configuration.boundary_crossing = false;
  configuration.land_crossing = false;

  wxPlugin_WaypointListNode* pwpnode = proute->pWaypointList->GetFirst();
  PlugIn_Waypoint* pwp;
  wxDateTime curtime;

  RoutePoint rte, *next;
  PlotData data;
  data.time = configuration.StartTime;
  curtime = data.time;
  double dt = configuration.DeltaTime;  // UsedDeltaTime;
  // sog, cog, stw, ctw, VW, W, tws, twd, currentSpeed, currentDir, WVHT;
  // double VW_GUST;
  data.WVHT = 0;
  data.VW_GUST = 0;
  data.delta = dt;
  bool ok = true;
  data.lat = configuration.StartLat, data.lon = configuration.StartLon;
  while (pwpnode) {
    pwp = pwpnode->GetData();
    configuration.time = data.time;
    data.lat = pwp->m_lat, data.lon = pwp->m_lon;
    double eta = dt;
    pwpnode = pwpnode->GetNext();  // PlugInWaypoint
    if (pwpnode == nullptr) break;

    DataMask data_mask = DataMask::NONE;
    double H;
    pwp = pwpnode->GetData();
    rte.lat = pwp->m_lat, rte.lon = pwp->m_lon;
    next = &rte;
    eta = data.PropagateToPoint(rte.lat, rte.lon, configuration, H, data_mask,
                                false);
    if (std::isnan(eta)) {
      ok = false;
      eta = dt;
    }
    // ll_gc_ll_reverse(data.lat, data.lon, next->lat, next->lon, &data.cog,
    // &data.sog);
    curtime += wxTimeSpan(0, 0, eta);
    if (configuration.wind_data_status == wxEmptyString) {
      data.GetPlotData(next, eta, configuration, data);
      plotdata.push_back(data);
    }
    if (!ok) break;
    data.time = curtime;
  }
  Lock();
  m_bUpdated = true;
  m_UpdateOverlay = true;
  last_destination_position =
      new Position(data.lat, data.lon, nullptr /* position */,
                   NAN /* heading */, NAN /* bearing*/, data.polar,
                   0 /* tacks */, 0 /* jibes */, 0 /* sailplan changes */,
                   DataMask::NONE /* data_mask */, true /* data_deficient */);

  last_cursor_plotdata = last_destination_plotdata;
  if (ok) {
    m_EndTime = data.time;
  }
  SetFinished(ok);
  UpdateStatus(configuration);
  Unlock();
}

void RouteMapOverlay::DeleteThread() {
  if (!m_Thread) return;

  m_Thread->Delete();
  delete m_Thread;
  m_Thread = nullptr;
}

static void SetColor(piDC& dc, wxColour c, bool penifgl = false) {
#ifndef __OCPN__ANDROID__
  if (!dc.GetDC()) {
    glColor4ub(c.Red(), c.Green(), c.Blue(), c.Alpha());
    if (!penifgl) return;
  }
#endif
  wxPen pen = dc.GetPen();
  pen.SetColour(c);
  dc.SetPen(pen);
}

static void SetWidth(piDC& dc, int w, bool penifgl = false) {
  if (!dc.GetDC()) {
    glLineWidth(w);
    if (!penifgl) return;
  }
  wxPen pen = dc.GetPen();
  pen.SetWidth(w);
  dc.SetPen(pen);
}

void RouteMapOverlay::DrawLine(RoutePoint* p1, RoutePoint* p2, piDC& dc,
                               PlugIn_ViewPort& vp) {
  wxPoint p1p, p2p;
  WR_GetCanvasPixLL(&vp, &p1p, p1->lat, p1->lon);
  WR_GetCanvasPixLL(&vp, &p2p, p2->lat, p2->lon);

#ifndef __OCPN__ANDROID__
  if (!dc.GetDC()) {
    glVertex2d(p1p.x, p1p.y);
    glVertex2d(p2p.x, p2p.y);
  } else
#endif
  {
    dc.StrokeLine(p1p.x, p1p.y, p2p.x, p2p.y);
  }
}

void RouteMapOverlay::DrawLine(RoutePoint* p1, wxColour& color1, RoutePoint* p2,
                               wxColour& color2, piDC& dc,
                               PlugIn_ViewPort& vp) {
#if 0
    double p1plon, p2plon;
    if(fabs(vp.clon) > 90)
        p1plon = positive_degrees(p1->lon), p2plon = positive_degrees(p2->lon);
    else
        p1plon = heading_resolve(p1->lon), p2plon = heading_resolve(p2->lon);

    if((p1plon+180 < vp.clon && p2plon+180 > vp.clon) ||
       (p1plon+180 > vp.clon && p2plon+180 < vp.clon) ||
       (p1plon-180 < vp.clon && p2plon-180 > vp.clon) ||
       (p1plon-180 > vp.clon && p2plon-180 < vp.clon))
        return;
#endif

  wxPoint p1p, p2p;
  WR_GetCanvasPixLL(&vp, &p1p, p1->lat, p1->lon);
  WR_GetCanvasPixLL(&vp, &p2p, p2->lat, p2->lon);

  SetColor(dc, color1);
#ifndef __OCPN__ANDROID__
  if (!dc.GetDC()) {
    glVertex2d(p1p.x, p1p.y);
    SetColor(dc, color2);
    glVertex2d(p2p.x, p2p.y);
  } else
#endif
  {
    dc.DrawLine(p1p.x, p1p.y, p2p.x, p2p.y);
  }
}

static inline wxColour& PositionColor(Position* p, wxColour& grib_color,
                                      wxColour& climatology_color,
                                      wxColour& grib_deficient_color,
                                      wxColour& climatology_deficient_color) {
  if (p->data_mask & DataMask::GRIB_WIND) {
    if (p->data_mask & DataMask::DATA_DEFICIENT_WIND)
      return grib_deficient_color;
    else
      return grib_color;
  }

  if (p->data_mask & DataMask::CLIMATOLOGY_WIND) {
    if (p->data_mask & DataMask::DATA_DEFICIENT_WIND)
      return climatology_deficient_color;
    else
      return climatology_color;
  }

  static wxColour black(0, 0, 0);
  return black;
}

static wxColour TransparentColor(wxColor c) {
  return wxColor(c.Red(), c.Green(), c.Blue(), c.Alpha() * 7 / 24);
}

void RouteMapOverlay::RenderIsoRoute(IsoRoute* r, wxDateTime time,
                                     wxColour& grib_color,
                                     wxColour& climatology_color, piDC& dc,
                                     PlugIn_ViewPort& vp) {
  SkipPosition* s = r->skippoints;
  if (!s) return;

  wxColour grib_deficient_color = TransparentColor(grib_color);
  wxColour climatology_deficient_color = TransparentColor(climatology_color);

  Position* p = s->point;
  wxColour* pcolor =
      &PositionColor(p, grib_color, climatology_color, grib_deficient_color,
                     climatology_deficient_color);
#ifndef __OCPN__ANDROID__
  if (!dc.GetDC()) glBegin(GL_LINES);
#endif
  do {
    wxColour& ncolor =
        PositionColor(p->next, grib_color, climatology_color,
                      grib_deficient_color, climatology_deficient_color);
    if (!p->copied || !p->next->copied)
      DrawLine(p, *pcolor, p->next, ncolor, dc, vp);
    pcolor = &ncolor;
    p = p->next;
  } while (p != s->point);

#ifndef __OCPN__ANDROID__
  if (!dc.GetDC()) glEnd();
#endif
  /* now render any children */
  wxColour cyan(0, 255, 255), magenta(255, 0, 255);
  for (IsoRouteList::iterator it = r->children.begin(); it != r->children.end();
       ++it)
    RenderIsoRoute(*it, time, cyan, magenta, dc, vp);
}

void RouteMapOverlay::RenderAlternateRoute(IsoRoute* r, bool each_parent,
                                           piDC& dc, PlugIn_ViewPort& vp) {
  Position* pos = r->skippoints->point;
  wxColor black = wxColour(0, 0, 0, 192), tblack = TransparentColor(black);
  do {
    wxColour* color =
        pos->data_mask & DataMask::DATA_DEFICIENT_WIND ? &tblack : &black;
    for (Position* p = pos; p && !p->drawn && p->parent; p = p->parent) {
      //            wxColour &color = p->data_mask &
      //            DataMask::DATA_DEFICIENT_WIND ? tblack : black;
      wxColour& pcolor =
          p->parent->data_mask & DataMask::DATA_DEFICIENT_WIND ? tblack : black;
      if (!p->copied || each_parent)
        DrawLine(p, *color, p->parent, pcolor, dc, vp);
      p->drawn = true;
      if (!each_parent) break;
      color = &pcolor;
    }

    pos = pos->next;
  } while (pos != r->skippoints->point);

  wxColour blue(0, 0, 255);
  SetColor(dc, blue);
  for (IsoRouteList::iterator cit = r->children.begin();
       cit != r->children.end(); cit++)
    RenderAlternateRoute(*cit, each_parent, dc, vp);
}

static wxColour Darken(wxColour c) {
  return wxColour(c.Red() * 2 / 3, c.Green() * 2 / 3, c.Blue() * 2 / 3,
                  c.Alpha());
}

static double GetPlatformScaleFactor() {
  double scale_factor = OCPN_GetDisplayContentScaleFactor();
#ifdef __WXMSW__
  scale_factor *= OCPN_GetWinDIPScaleFactor();
#endif
  return scale_factor;
}

void RouteMapOverlay::Render(wxDateTime time, SettingsDialog& settingsdialog,
                             piDC& dc, PlugIn_ViewPort& vp, bool justendroute,
                             RoutePoint* positionOnRoute) {
  dc.SetPen(*wxBLACK);                // reset pen
  dc.SetBrush(*wxTRANSPARENT_BRUSH);  // reset brush
  if (!justendroute) {
    RouteMapConfiguration configuration = GetConfiguration();

    if (!std::isnan(configuration.StartLat)) {
      wxPoint r;
      WR_GetCanvasPixLL(&vp, &r, configuration.StartLat,
                        configuration.StartLon);
      SetColor(dc, *wxBLUE, true);
      SetWidth(dc, 3, true);
      dc.DrawLine(r.x, r.y - 10, r.x + 10, r.y + 7);
      dc.DrawLine(r.x, r.y - 10, r.x - 10, r.y + 7);
      dc.DrawLine(r.x - 10, r.y + 7, r.x + 10, r.y + 7);
    }

    if (!std::isnan(configuration.EndLat)) {
      wxPoint r;
      WR_GetCanvasPixLL(&vp, &r, configuration.EndLat, configuration.EndLon);
      SetColor(dc, *wxRED, true);
      SetWidth(dc, 3, true);
      dc.DrawLine(r.x - 10, r.y - 10, r.x + 10, r.y + 10);
      dc.DrawLine(r.x - 10, r.y + 10, r.x + 10, r.y - 10);
    }

    static const double NORM_FACTOR = 16;

    // Do not use displaylist processing to avoid incorrect vp calculations in
    // O562+
    bool use_dl = false;  // vp.m_projection_type == PI_PROJECTION_MERCATOR;
#ifndef __OCPN__ANDROID__
    if (!dc.GetDC() && use_dl) {
      glPushMatrix();

      /* center display list on start lat/lon */

      wxPoint point;
      WR_GetCanvasPixLL(&vp, &point, configuration.StartLat,
                        configuration.StartLon);

      glTranslated(point.x, point.y, 0);
      glScalef(vp.view_scale_ppm / NORM_FACTOR, vp.view_scale_ppm / NORM_FACTOR,
               1);
      glRotated(vp.rotation * 180 / M_PI, 0, 0, 1);
    }

    if (!dc.GetDC() && !m_UpdateOverlay && use_dl &&
        vp.m_projection_type == m_overlaylist_projection) {
      glCallList(m_overlaylist);
      glPopMatrix();

    } else
#endif
    {
      PlugIn_ViewPort nvp = vp;

#ifndef __OCPN__ANDROID__
      if (!dc.GetDC() && use_dl) {
        m_UpdateOverlay = false;

        if (!m_overlaylist) m_overlaylist = glGenLists(1);

        glNewList(m_overlaylist, GL_COMPILE);

        nvp.clat = configuration.StartLat, nvp.clon = configuration.StartLon;
        nvp.pix_width = nvp.pix_height = 0;
        nvp.view_scale_ppm = NORM_FACTOR;
        nvp.rotation = nvp.skew = 0;

        m_overlaylist_projection = vp.m_projection_type;
      }
#endif
      /* draw alternate routes first */
      int AlternateRouteThickness =
          settingsdialog.m_sAlternateRouteThickness->GetValue();
      if (AlternateRouteThickness) {
        Lock();
        IsoChronList::iterator it;

        /* reset drawn flag for all positions
           this is used to avoid duplicating alternate route segments */
        for (it = origin.begin(); it != origin.end(); ++it)
          (*it)->ResetDrawnFlag();

        bool AlternatesForAll = settingsdialog.m_cbAlternatesForAll->GetValue();
        if (AlternatesForAll)
          it = origin.begin();
        else {
          it = origin.end();
          it--;
        }

        SetWidth(dc, AlternateRouteThickness);
#ifndef __OCPN__ANDROID__
        if (!dc.GetDC()) glBegin(GL_LINES);
#endif
        for (; it != origin.end(); ++it)
          for (IsoRouteList::iterator rit = (*it)->routes.begin();
               rit != (*it)->routes.end(); ++rit) {
            RenderAlternateRoute(*rit, !AlternatesForAll, dc, nvp);
          }

#ifndef __OCPN__ANDROID__
        if (!dc.GetDC()) glEnd();
#endif
        Unlock();
      }

      static const unsigned char routecolors[][3] = {
          {0, 0, 128},   {0, 192, 0},   {0, 128, 192}, {0, 255, 0},
          {0, 0, 255},   {0, 128, 128}, {0, 255, 0},   {0, 192, 192},
          {0, 128, 255}, {0, 255, 128}, {0, 0, 255},   {0, 192, 0},
          {0, 0, 128},   {0, 255, 0},   {0, 192, 128}, {0, 128, 255},
          {0, 192, 0},   {0, 128, 0},   {0, 0, 255},   {0, 192, 192}};
#if 0
                {255, 127,   0}, {255, 127, 127},
                {  0, 255,   0}, {  0, 255, 127},
                {127, 255,   0}, {127, 255, 127},
                {127, 127,   0},                  {127, 127, 255},
                {255,   0,   0}, {255,   0, 127}, {255,   0, 255},
                {127,   0,   0}, {127,   0, 127}, {127,   0, 255},
                {  0, 127,   0}, {  0, 127, 127}, {  0, 127, 255},
                {255, 255,   0},                  };
#endif

      int IsoChronThickness = settingsdialog.m_sIsoChronThickness->GetValue();
      if (IsoChronThickness) {
        Lock();
        int c = 0;
        // Find the isochron closest to the GRIB time
        IsoChron* closestIsochron = nullptr;
        wxTimeSpan closestDiff =
            wxTimeSpan::Days(999);  // A large initial value
        if (time.IsValid()) {
          for (IsoChronList::iterator i = origin.begin(); i != origin.end();
               ++i) {
            wxTimeSpan diff = (*i)->time - time;
            if (diff.GetValue() < 0) {
              diff = -diff;
            }
            if (diff < closestDiff) {
              closestDiff = diff;
              closestIsochron = *i;
            }
          }
        }
        for (IsoChronList::iterator i = origin.begin(); i != origin.end();
             ++i) {
          Unlock();
          wxColor grib_color(routecolors[c][0], routecolors[c][1],
                             routecolors[c][2], 224);
          wxColor climatology_color(255 - routecolors[c][0], routecolors[c][2],
                                    routecolors[c][1], 224);
          // If this is the closest isochron to the selected GRIB time, use a
          // thicker line
          if (time.IsValid() && *i == closestIsochron) {
            SetWidth(dc, IsoChronThickness * 3);
          } else {
            SetWidth(dc, IsoChronThickness);
          }
          for (IsoRouteList::iterator j = (*i)->routes.begin();
               j != (*i)->routes.end(); ++j)
            RenderIsoRoute(*j, time, grib_color, climatology_color, dc, nvp);

          if (++c == (sizeof routecolors) / (sizeof *routecolors)) c = 0;
          Lock();
        }
        Unlock();
      }

#ifndef __OCPN__ANDROID__
      if (!dc.GetDC() && use_dl) {
        glEndList();
        glCallList(m_overlaylist);
        glPopMatrix();
      }
#endif
    }
  }

  int RouteThickness = settingsdialog.m_sRouteThickness->GetValue();
  if (RouteThickness) {
    wxColour CursorColor = settingsdialog.m_cpCursorRoute->GetColour(),
             DestinationColor =
                 settingsdialog.m_cpDestinationRoute->GetColour();
    bool MarkAtPolarChange = settingsdialog.m_cbMarkAtPolarChange->GetValue();

    if (!justendroute && settingsdialog.m_cbDisplayCursorRoute->GetValue()) {
      SetColor(dc, CursorColor, true);
      SetWidth(dc, RouteThickness, true);
      RenderCourse(true, dc, vp);

      if (MarkAtPolarChange) {
        SetColor(dc, Darken(CursorColor), true);
        SetWidth(dc, (RouteThickness + 1) / 2, true);
        RenderPolarChangeMarks(true, dc, vp);
      }
    }
    SetColor(dc, DestinationColor, true);
    SetWidth(dc, RouteThickness, true);
    bool confortOnRoute = settingsdialog.m_cbDisplayComfort->GetValue();
    RenderCourse(false, dc, vp, confortOnRoute);
    SetColor(dc, Darken(DestinationColor), true);
    SetWidth(dc, RouteThickness / 2, true);
    RenderBoatOnCourse(false, time, dc, vp);

    // Start WindBarbsOnRoute customization
    int lineWidth = settingsdialog.m_sWindBarbsOnRouteThickness->GetValue();
    bool apparent = settingsdialog.m_cbDisplayApparentWindBarbs->GetValue();
    if (lineWidth > 0) RenderWindBarbsOnRoute(dc, vp, lineWidth, apparent);

    // CUSTOMIZATION
    // Display the position of the cursor on route
    // where the infos are read from Route Position window
    if (positionOnRoute != nullptr) {
      wxPoint r;
      WR_GetCanvasPixLL(&vp, &r, positionOnRoute->lat, positionOnRoute->lon);
      wxColour ownBlue(20, 83, 186);
      SetColor(dc, ownBlue, true);
      SetWidth(dc, RouteThickness, true);
      double circle_size = 10;  // logical pixels.
      dc.StrokeCircle(r.x, r.y, circle_size);
    }

    if (MarkAtPolarChange) {
      SetColor(dc, Darken(DestinationColor), true);
      SetWidth(dc, (RouteThickness + 1) / 2, true);
      RenderPolarChangeMarks(false, dc, vp);
    }
  }
}

void RouteMapOverlay::RenderPolarChangeMarks(bool cursor_route, piDC& dc,
                                             PlugIn_ViewPort& vp) {
  Position* pos =
      cursor_route ? last_cursor_position : last_destination_position;

  if (!pos) return;

  std::list<PlotData> plot = GetPlotData(cursor_route);
  std::list<PlotData>::iterator itt = plot.begin();
  if (itt == plot.end()) {
    return;
  }

#ifndef __OCPN__ANDROID__
  if (!dc.GetDC()) glBegin(GL_LINES);
#endif

  int polar = itt->polar;
  for (; itt != plot.end(); itt++) {
    if (itt->polar == polar) continue;
    wxPoint r;
    WR_GetCanvasPixLL(&vp, &r, itt->lat, itt->lon);
    int s = 6;
#ifndef __OCPN__ANDROID__
    if (!dc.GetDC()) {
      glVertex2i(r.x - s, r.y - s), glVertex2i(r.x + s, r.y - s);
      glVertex2i(r.x + s, r.y - s), glVertex2i(r.x + s, r.y + s);
      glVertex2i(r.x + s, r.y + s), glVertex2i(r.x - s, r.y + s);
      glVertex2i(r.x - s, r.y + s), glVertex2i(r.x - s, r.y - s);
    } else
#endif
      dc.DrawRectangle(r.x - s, r.y - s, 2 * s, 2 * s);
    polar = itt->polar;
  }
#ifndef __OCPN__ANDROID__
  if (!dc.GetDC()) glEnd();
#endif
}

/* Customization ComfortDisplay
 * ----------------------------------------------------------------------------
 * The idea is to display on the weather route different colors giving an idea
 * of the sailing comfort along the trip:
 *    Green = Light conditions, relax and enjoy
 *    Orange = Can be tough, stay focus
 *    Red = Strong conditions, heavy sailors, be prepared
 */
int RouteMapOverlay::sailingConditionLevel(const PlotData& plot) const {
  /* Method to calculate a indicator between 1 and 3 of the sailing conditions
   * based on wind, wind course and waves.
   *
   * All these calculations are empirical and just made from experience and how
   * people feel sailing comfort which is a highly subjective value...
   */

  double level_calc = 0.0;

  // Define maximum constants. Over this value, sailing comfort is very impacted
  // (coef > 1) and automatically displayed in red.
  // Definitions:
  // AW   - Apparent Wind Direction from the boat (0 = upwind)
  // VW   - Velocity of wind over water
  // WVHT - Swell (if available)
  double MAX_WV = 27;   // Vigilant over 27knts == 7B
  double MAX_AW = 35;   // Upwind start at 35° from wind
  double MAX_WVHT = 5;  // No more than 5m waves

  // Wind impact exponentially on sailing comfort
  // We propose a power 3 function as difficulties increase exponentially
  // Over 30knts, it starts to be tough
  double twsOverWater = plot.twsOverWater;
  double WV_normal = pow(twsOverWater / MAX_WV, 3);

  // Wind direction impact on sailing comfort.
  // Ex: if you decide to sail upwind with 30knts, it is not the same
  // conditions as if you sail downwind (impact of waves, heel, and more).
  // Use a normal distribution to set the maximum difficulty at 35° upwind,
  // and reduce when we go downwind.
  double AW = heading_resolve(plot.ctw - plot.twdOverWater);
  double teta = 30;
  double mu = 35;
  double amp = 20;
  double AW_normal = amp * (1 / (teta * pow((2 * M_PI), 0.5))) *
                     exp(-pow(AW - mu, 2) / (2 * pow(teta, 2)));

  // If available, add swell conditions in comfort model.
  // Use same exponential function for swell as sailing
  // comfort exponentially decrease with swell height.
  double WVHT = plot.WVHT;
  double WVHT_normal = 0.0;
  if (WVHT > 0) WVHT_normal = pow(WVHT / MAX_WVHT, 2);

  // Calculate score
  // Use an OR function X,Y E [0,1], f(X,Y) = 1-(1-X)(1-Y)
  level_calc = 1 - (1 - WV_normal * (1 + AW_normal) * (1 + WVHT_normal));

  if (level_calc <= 0.5)
    // Light conditions, enjoy ;-)
    return 1;
  if (level_calc > 0.5 && level_calc < 1)
    // Can be tough
    return 2;
  if (level_calc >= 1)
    // Strong conditions
    return 3;
  return 0;
}

wxColour RouteMapOverlay::sailingConditionColor(int level) {
  switch (level) {
    case 1:
      return wxColor(50, 205, 50);
    case 2:
      return wxColor(255, 165, 0);
    case 3:
      return *wxRED;
  }
  return *wxBLACK;
}

wxString RouteMapOverlay::sailingConditionText(int level) {
  if (level == 1) return _("Good");
  if (level == 2) return _("Bumpy");
  if (level == 3) return _("Difficult");
  return _("N/A");
}

// -----------------------------------------------------

void RouteMapOverlay::RenderCourse(bool cursor_route, piDC& dc,
                                   PlugIn_ViewPort& vp, bool comfortRoute) {
  Position* pos =
      cursor_route ? last_cursor_position : last_destination_position;
  if (!pos) return;

  Lock();

  bool rte = !GetConfiguration().RouteGUID.IsEmpty();
  if (cursor_route == true) {
    // never draw comfort if cursor route
    assert(comfortRoute == false);
    if (!rte) {
#ifndef __OCPN__ANDROID__
      if (!dc.GetDC()) glBegin(GL_LINES);
#endif
      for (Position* p = pos; p && p->parent; p = p->parent)
        DrawLine(p, p->parent, dc, vp);
#ifndef __OCPN__ANDROID__
      if (!dc.GetDC()) glEnd();
#endif
    }
    Unlock();
    return;
  }
  Unlock();

  /* ComfortDisplay Customization
   * ------------------------------------------------
   * To get weather data (wind, current, waves) on a
   * position and through time, iterate over the
   * position and in parallel on GetPlotData
   * Thanks Sean for your help :-)
   */
  std::list<PlotData> plot = GetPlotData(false);
  std::list<PlotData>::reverse_iterator itt = plot.rbegin();
  std::list<PlotData>::reverse_iterator inext = itt;

  if (itt == plot.rend()) {
    return;
  }

  wxColor lc = sailingConditionColor(sailingConditionLevel(*itt));

  /* draw lines to this route */
#ifndef __OCPN__ANDROID__
  if (!dc.GetDC()) glBegin(GL_LINES);
#endif

  /* end point if reached is not in GetPlotData */
  RoutePoint *to, from;
  for (to = pos; itt != plot.rend(); inext = itt, itt++, to = &(*itt)) {
    RoutePoint* from = &(*inext);
    if (comfortRoute) {
      wxColor c = sailingConditionColor(sailingConditionLevel(*itt));
      DrawLine(to, c, from, lc, dc, vp);
      lc = c;
    } else {
      DrawLine(to, from, dc, vp);
    }
  }

#ifndef __OCPN__ANDROID__
  if (!dc.GetDC()) glEnd();
#endif
}

void RouteMapOverlay::RenderBoatOnCourse(bool cursor_route, wxDateTime time,
                                         piDC& dc, PlugIn_ViewPort& vp) {
  /* Dedicated method to render the boat circle
   * on the weather route to be able to select the
   * color of the maker, and avoid to generate twice
   * (1 normally, 1 for polar changed -- to avoid)
   */
  Position* pos =
      cursor_route ? last_cursor_position : last_destination_position;
  if (!pos) return;

  std::list<PlotData> plot = GetPlotData(cursor_route);

  for (auto it = plot.begin(); it != plot.end();) {
    wxDateTime ittime = it->time;

    wxDateTime timestart = ittime;
    double plat = it->lat;
    double plon = it->lon;
    it++;
    if (it == plot.end()) break;

    wxDateTime timeend = it->time;
    if (!(time >= timestart && time <= timeend)) continue;

    wxTimeSpan span = timeend - timestart, cspan = time - timestart;
    double d = cspan.GetSeconds().ToDouble() / span.GetSeconds().ToDouble();

    if (d > 1)
      // d = 1; // draw at end??
      break;  // don't draw if grib time is after end

    wxPoint r;
    WR_GetCanvasPixLL(&vp, &r, plat + d * (it->lat - plat),
                      plon + d * heading_resolve(it->lon - plon));

    SetWidth(dc, 8, true);
    double circle_size = 20;  // logical pixels
    dc.DrawCircle(r.x, r.y, circle_size);

    // Outer circle
    dc.SetPen(wxPen(*wxWHITE, 4));
    // dc.SetBrush(wxBrush(*wxTRANSPARENT_BRUSH));
    dc.DrawCircle(r.x, r.y, circle_size + 4);

    // Inner circle
    // dc.SetPen(wxPen(*wxBLACK, 1));
    dc.DrawCircle(r.x, r.y, circle_size - 4);
    break;
  }
}

void RouteMapOverlay::RenderWindBarbsOnRoute(piDC& dc, PlugIn_ViewPort& vp,
                                             int lineWidth, bool apparentWind) {
  /* Method to render wind barbs on the route that has been generated
   * by WeatherRouting plugin. The idead is to visualize the wind
   * direction and strength at any step of the trip.
   *
   * Customization by: Sylvain Carlioz -- with Pavel Kalian's help ;-)
   * OpenCPN's licence
   * March, 2018
   */

  if (vp.bValid == false) return;

  RouteMapConfiguration configuration = GetConfiguration();

  // Create a specific viewport at position (0,0)
  // to draw the winds barbs, and then translate it
  PlugIn_ViewPort nvp = vp;
  // calculate wind barbs along the route by looping
  // over [GetPlotData(false)] list which contains lat,
  // lon, wind info for each points, only if needed.
  std::list<PlotData> plot = GetPlotData(false);

  // if no route has been calculated by WeatherRouting,
  // then stops the method.
  if (plot.empty()) return;

  for (std::list<PlotData>::iterator it = plot.begin(); it != plot.end();
       it++) {
    wxPoint p;
    WR_GetCanvasPixLL(&nvp, &p, it->lat, it->lon);

    // available
    //   twd tws : winds over ground
    //   W VW : winds over water
    //   currentDir currentSpeed : current
    //
    //   cog sog : boat speed over ground
    //   ctw  stw  : boat speed over water
    float windSpeed = it->twsOverWater;
    float windDirection =
        it->twdOverWater;  // heading_resolve(it->ctw - it->W);

    // By default, display true wind
    float finalWindSpeed = windSpeed;
    float finalWindDirection = windDirection;

    if (apparentWind) {
      finalWindSpeed = Polar::VelocityApparentWind(
          it->stw, heading_resolve(it->ctw - windDirection), windSpeed);
      finalWindDirection = heading_resolve(
          it->ctw -
          Polar::DirectionApparentWind(finalWindSpeed, it->stw,
                                       heading_resolve(it->ctw - windDirection),
                                       it->twsOverWater));
    }

    // Draw barbs
    g_barbsOnRoute_LineBufferOverlay.setLineWidth(lineWidth);
    g_barbsOnRoute_LineBufferOverlay.pushWindArrowWithBarbs(
        wind_barb_route_cache, p.x, p.y, finalWindSpeed,
        deg2rad(finalWindDirection) + nvp.rotation, it->lat < 0, true);
  }
  wind_barb_route_cache.Finalize();

  // Draw the wind barbs
  wxPoint point;
  WR_GetCanvasPixLL(&vp, &point, configuration.StartLat,
                    configuration.StartLon);
  wxColour colour;
  if (apparentWind) {
    wxColour blue(20, 83, 186);
    colour = blue;
  } else {
    wxColour purple(170, 0, 170);
    colour = purple;
  }

  if (dc.GetDC()) {
    dc.SetPen(wxPen(colour, 2));
  }
#if defined(ocpnUSE_GL) && !defined(__OCPN__ANDROID__)
  else {
    // Mandatory to avoid display issue when moving map
    // (map disappear to show a gray background...)
    glPushMatrix();

    // Anti-aliasing options to render
    // wind barbs at best quality (copy from grip_pi)
    glEnable(GL_BLEND);
    glEnable(GL_LINE_SMOOTH);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);

    glColor3ub(colour.Red(), colour.Green(), colour.Blue());
    glLineWidth(lineWidth);
    glEnableClientState(GL_VERTEX_ARRAY);
  }
#endif

  wind_barb_route_cache.draw(dc.GetDC());

#if defined(ocpnUSE_GL) && !defined(__OCPN__ANDROID__)
  if (!dc.GetDC()) {
    glDisableClientState(GL_VERTEX_ARRAY);
    glPopMatrix();
  }
#endif
}

void RouteMapOverlay::RenderWindBarbs(piDC& dc, PlugIn_ViewPort& vp) {
  if (origin.size() < 2)  // no map to work with
    return;

  if (vp.bValid == false) return;

  RouteMapConfiguration configuration = GetConfiguration();

  // if zoomed way in, don't cache the arrows for panning, instead we just
  // render what's onscreen
  double latmin, latmax, lonmin, lonmax;
  GetLLBounds(latmin, latmax, lonmin, lonmax);

  PlugIn_ViewPort nvp = vp;
  nvp.clat = configuration.StartLat, nvp.clon = configuration.StartLon;
  nvp.pix_width = nvp.pix_height = 0;
  nvp.rotation = nvp.skew = 0;

  wxPoint p1, p2, p3, p4;
  WR_GetCanvasPixLL(&nvp, &p1, latmin, lonmin);
  WR_GetCanvasPixLL(&nvp, &p2, latmin, lonmax);
  WR_GetCanvasPixLL(&nvp, &p3, latmax, lonmin);
  WR_GetCanvasPixLL(&nvp, &p4, latmax, lonmax);

  wxRect r;
  r.x = wxMin(wxMin(p1.x, p2.x), wxMin(p3.x, p4.x));
  r.y = wxMin(wxMin(p1.y, p2.y), wxMin(p3.y, p4.y));
  r.width = wxMax(wxMax(p1.x, p2.x), wxMax(p3.x, p4.x)) - r.x;
  r.height = wxMax(wxMax(p1.y, p2.y), wxMax(p3.y, p4.y)) - r.y;

  // we could somehow "append" to the cache as passing occurs when zoomed really
  // far in rather than making a complete cache... but how complex does it need
  // to be? quick an dirty, convert to double or integer may overflow
  bool nocache = (double)r.width * (double)r.height >
                     (double)(vp.rv_rect.width * vp.rv_rect.height * 4) ||
                 vp.m_projection_type != PI_PROJECTION_MERCATOR;

  if (nocache || origin.size() != wind_barb_cache_origin_size ||
      vp.view_scale_ppm != wind_barb_cache_scale ||
      vp.m_projection_type != wind_barb_cache_projection) {
    wxStopWatch timer;
    static double step = 36.0;

    wind_barb_cache_origin_size = origin.size();
    wind_barb_cache_scale = vp.view_scale_ppm;
    wind_barb_cache_projection = vp.m_projection_type;

    if (nocache) {
      r = vp.rv_rect;
      nvp = vp;
    }

    Lock();

    wxPoint p;
    WR_GetCanvasPixLL(&nvp, &p, configuration.StartLat, configuration.StartLon);
    int xoff = p.x % (int)step, yoff = p.y % (int)step;

    IsoChronList::iterator it = origin.end();
    it--;
    for (double x = r.x + xoff; x < r.x + r.width; x += step) {
      for (double y = r.y + yoff; y < r.y + r.height; y += step) {
        double lat, lon;
        GetCanvasLLPix(&nvp, wxPoint(x, y), &lat, &lon);

        Position p(lat, configuration.positive_longitudes
                            ? positive_degrees(lon)
                            : lon);

        // find the first isochron we are outside of using the isochron from
        // the last point as an initial guess to reduce the amount of expensive
        // Contains calls
        if (!(*it)->Contains(p)) {
          do {
            if (++it == origin.end()) {  // don't plot outside map
              it--;
              goto skip;
            }
          } while (!(*it)->Contains(p));
          it--;
        } else
          for (it--; it != origin.begin(); it--)
            if (!(*it)->Contains(p)) break;

        {
          double W1, VW1, W2, VW2;
          DataMask data_mask1,
              data_mask2;  // can be used to colorize barbs based on data type
          bool v1, v2;
          // now it is the isochron before p, so we find the two closest
          // postions
          Position* p1 = (*it)->ClosestPosition(lat, lon);
          configuration.grib = (*it)->m_Grib;
          configuration.time = (*it)->time;
          configuration.grib_is_data_deficient =
              (*it)->m_Grib_is_data_deficient;
          p.grib_is_data_deficient = configuration.grib_is_data_deficient;
          v1 = p.GetWindData(configuration, W1, VW1, data_mask1);

          it++;
          Position* p2 = (*it)->ClosestPosition(lat, lon);
          configuration.grib = (*it)->m_Grib;
          configuration.time = (*it)->time;
          configuration.grib_is_data_deficient =
              (*it)->m_Grib_is_data_deficient;
          p.grib_is_data_deficient = configuration.grib_is_data_deficient;
          v2 = p.GetWindData(configuration, W2, VW2, data_mask2);
          if (!v1 || !v2) {
            // not valid data
            goto skip;
          }
          // now polar interpolation of the two wind positions
          double d1 = p.Distance(p1), d2 = p.Distance(p2);
          double d = d1 / (d1 + d2);
#if 0
                double W1r = deg2rad(W1), W2r = deg2rad(W2);
                double W1x = VW1*cos(W1r), W1y = VW1*sin(W1r);
                double W2x = VW2*cos(W2r), W2y = VW2*sin(W2r);
                double Wx = d*W1x + (1-d)*W2x, Wy = d*W1y + (1-d)*W2y;
                double W = rad2deg(atan2(Wy, Wx));
#else
          while (W1 - W2 > 180) W1 -= 360;
          while (W2 - W1 > 180) W2 -= 360;
          double W = d * W1 + (1 - d) * W2;
#endif
          double VW = d * VW1 + (1 - d) * VW2;

          g_LineBufferOverlay.pushWindArrowWithBarbs(
              wind_barb_cache, x, y, VW, deg2rad(W) + nvp.rotation, lat < 0);
        }
      skip:;
      }
    }

    Unlock();

    // evaluate performance, and "cheat" by spacing the barbes more in
    // subsequent frames if peformance is inadequate
    long time = timer.Time();
    if (nocache && time > 100 &&
        step < 300)  // 100 milliseconds is unacceptable per frame
      step *= 1.5;
    else if (time < 10 && step > 40)  // reset step
      step /= 1.5;

    wind_barb_cache.Finalize();
  }

  wxColour colour(180, 140, 14);

  wxPoint point;
  WR_GetCanvasPixLL(&vp, &point, configuration.StartLat,
                    configuration.StartLon);

  if (dc.GetDC()) dc.SetPen(wxPen(colour, 2));
#if defined(ocpnUSE_GL) && !defined(__OCPN__ANDROID__)
  else {
    if (!nocache) {
      glPushMatrix();
      glTranslated(point.x, point.y, 0);
      glRotated(vp.rotation * 180 / M_PI, 0, 0, 1);
    }

    glColor3ub(colour.Red(), colour.Green(), colour.Blue());
    //      Enable anti-aliased lines, at best quality
    glEnable(GL_BLEND);
    glLineWidth(2);
    glEnableClientState(GL_VERTEX_ARRAY);
  }
#endif

  if (dc.GetDC()) {
    if (nocache)
      wind_barb_cache.draw(dc.GetDC());
    else {
      LineBuffer tb;
      tb.pushTransformedBuffer(wind_barb_cache, point.x, point.y, vp.rotation);
      tb.Finalize();
      tb.draw(dc.GetDC());
    }
  } else
    wind_barb_cache.draw(nullptr);

#if defined(ocpnUSE_GL) && !defined(__OCPN__ANDROID__)
  if (!dc.GetDC()) {
    glDisableClientState(GL_VERTEX_ARRAY);

    if (!nocache) glPopMatrix();
  }
#endif
}

void RouteMapOverlay::RenderCurrent(piDC& dc, PlugIn_ViewPort& vp) {
  if (origin.size() < 2)  // no map to work with
    return;

  if (vp.bValid == false) return;

  RouteMapConfiguration configuration = GetConfiguration();

  // if zoomed way in, don't cache the arrows for panning, instead we just
  // render what's onscreen
  double latmin, latmax, lonmin, lonmax;
  GetLLBounds(latmin, latmax, lonmin, lonmax);

  PlugIn_ViewPort nvp = vp;
  nvp.clat = configuration.StartLat, nvp.clon = configuration.StartLon;
  nvp.pix_width = nvp.pix_height = 0;
  nvp.rotation = nvp.skew = 0;

  wxPoint p1, p2, p3, p4;
  WR_GetCanvasPixLL(&nvp, &p1, latmin, lonmin);
  WR_GetCanvasPixLL(&nvp, &p2, latmin, lonmax);
  WR_GetCanvasPixLL(&nvp, &p3, latmax, lonmin);
  WR_GetCanvasPixLL(&nvp, &p4, latmax, lonmax);

  wxRect r;
  r.x = wxMin(wxMin(p1.x, p2.x), wxMin(p3.x, p4.x));
  r.y = wxMin(wxMin(p1.y, p2.y), wxMin(p3.y, p4.y));
  r.width = wxMax(wxMax(p1.x, p2.x), wxMax(p3.x, p4.x)) - r.x;
  r.height = wxMax(wxMax(p1.y, p2.y), wxMax(p3.y, p4.y)) - r.y;

  // we could somehow "append" to the cache as passing occurs when zoomed really
  // far in rather than making a complete cache... but how complex does it need
  // to be? quick an dirty, convert to double or integer may overflow
  bool nocache = (double)r.width * (double)r.height >
                     (double)(vp.rv_rect.width * vp.rv_rect.height * 9) ||
                 vp.m_projection_type != PI_PROJECTION_MERCATOR;

  if (nocache || origin.size() != current_cache_origin_size ||
      vp.view_scale_ppm != current_cache_scale ||
      vp.m_projection_type != current_cache_projection) {
    wxStopWatch timer;
    static double step = 80.0;

    current_cache_origin_size = origin.size();
    current_cache_scale = vp.view_scale_ppm;
    current_cache_projection = vp.m_projection_type;

    if (nocache) {
      r = vp.rv_rect;
      nvp = vp;
    }

    Lock();

    wxPoint p;
    WR_GetCanvasPixLL(&nvp, &p, configuration.StartLat, configuration.StartLon);
    int xoff = p.x % (int)step, yoff = p.y % (int)step;

    IsoChronList::iterator it = origin.end();
    it--;
    for (double x = r.x + xoff; x < r.x + r.width; x += step) {
      for (double y = r.y + yoff; y < r.y + r.height; y += step) {
        double lat, lon;
        GetCanvasLLPix(&nvp, wxPoint(x, y), &lat, &lon);

        Position p(lat, configuration.positive_longitudes
                            ? positive_degrees(lon)
                            : lon);

        // find the first isochron we are outside of using the isochron from
        // the last point as an initial guess to reduce the amount of expensive
        // Contains calls
        if (!(*it)->Contains(p)) {
          do {
            if (++it == origin.end()) {  // don't plot outside map
              it--;
              goto skip;
            }
          } while (!(*it)->Contains(p));
          it--;
        } else
          for (it--; it != origin.begin(); it--)
            if (!(*it)->Contains(p)) break;

        {
          double W1, VW1, W2, VW2;
          DataMask data_mask1,
              data_mask2;  // can be used to colorize barbs based on data type

          // now it is the isochron before p, so we find the two closest
          // postions
          Position* p1 = (*it)->ClosestPosition(lat, lon);
          configuration.grib = (*it)->m_Grib;
          configuration.time = (*it)->time;
          configuration.grib_is_data_deficient =
              (*it)->m_Grib_is_data_deficient;
          p.grib_is_data_deficient = configuration.grib_is_data_deficient;
          bool v1, v2;

          v1 = p.GetCurrentData(configuration, W1, VW1, data_mask1);

          it++;
          Position* p2 = (*it)->ClosestPosition(lat, lon);
          configuration.grib = (*it)->m_Grib;
          configuration.time = (*it)->time;
          configuration.grib_is_data_deficient =
              (*it)->m_Grib_is_data_deficient;
          p.grib_is_data_deficient = configuration.grib_is_data_deficient;
          v2 = p.GetCurrentData(configuration, W2, VW2, data_mask2);
          if (!v1 || !v2) {
            goto skip;
          }
#if 0
                // XX climatology angle is to not from
                if ((data_mask1 & Position::CLIMATOLOGY_CURRENT))
                    W1 += 180.0;
                if ((data_mask2 & Position::CLIMATOLOGY_CURRENT))
                    W2 += 180.0;
#endif

          // now polar interpolation of the two wind positions
          double d1 = p.Distance(p1), d2 = p.Distance(p2);
          double d = d1 / (d1 + d2);
#if 0
                double W1r = deg2rad(W1), W2r = deg2rad(W2);
                double W1x = VW1*cos(W1r), W1y = VW1*sin(W1r);
                double W2x = VW2*cos(W2r), W2y = VW2*sin(W2r);
                double Wx = d*W1x + (1-d)*W2x, Wy = d*W1y + (1-d)*W2y;
                double W = rad2deg(atan2(Wy, Wx));
#else
          while (W1 - W2 > 180) W1 -= 360;
          while (W2 - W1 > 180) W2 -= 360;
          double W = d * W1 + (1 - d) * W2;
#endif
          double VW = d * VW1 + (1 - d) * VW2;

          g_LineBufferOverlay.pushSingleArrow(current_cache, x, y, VW,
                                              deg2rad(W + 180) + nvp.rotation,
                                              lat < 0);
        }
      skip:;
      }
    }

    Unlock();

    // evaluate performance, and "cheat" by spacing the barbes more in
    // subsequent frames if peformance is inadequate
    long time = timer.Time();
    if (nocache && time > 100 &&
        step < 600.)  // 100 milliseconds is unacceptable per frame
      step *= 1.5;
    else if (time < 10 && step > 90.)  // reset step
      step /= 1.5;

    current_cache.Finalize();
  }

  wxColour colour(0, 0, 0);

  wxPoint point;
  WR_GetCanvasPixLL(&vp, &point, configuration.StartLat,
                    configuration.StartLon);

  if (dc.GetDC()) dc.SetPen(wxPen(colour, 2));
#if defined(ocpnUSE_GL) && !defined(__OCPN__ANDROID__)
  else {
    if (!nocache) {
      glPushMatrix();
      glTranslated(point.x, point.y, 0);
      glRotated(vp.rotation * 180 / M_PI, 0, 0, 1);
    }

    glColor3ub(colour.Red(), colour.Green(), colour.Blue());
    //      Enable anti-aliased lines, at best quality
    glEnable(GL_BLEND);
    glLineWidth(2);
    glEnableClientState(GL_VERTEX_ARRAY);
  }
#endif

  if (dc.GetDC()) {
    if (nocache)
      current_cache.draw(dc.GetDC());
    else {
      LineBuffer tb;
      tb.pushTransformedBuffer(current_cache, point.x,
                               dc.GetDC()->GetSize().y - point.y, vp.rotation);
      tb.Finalize();
      tb.draw(dc.GetDC());
    }
  } else
    current_cache.draw(nullptr);

#if defined(ocpnUSE_GL) && !defined(__OCPN__ANDROID__)
  if (!dc.GetDC()) {
    glDisableClientState(GL_VERTEX_ARRAY);

    if (!nocache) glPopMatrix();
  }
#endif
}

void RouteMapOverlay::GetLLBounds(double& latmin, double& latmax,
                                  double& lonmin, double& lonmax) {
  latmin = INFINITY, lonmin = INFINITY;
  latmax = -INFINITY, lonmax = -INFINITY;

  IsoChron* last = origin.back();
  for (IsoRouteList::iterator it = last->routes.begin();
       it != last->routes.end(); ++it) {
    Position* pos = (*it)->skippoints->point;
    do {
      latmin = wxMin(latmin, pos->lat);
      latmax = wxMax(latmax, pos->lat);
      lonmin = wxMin(lonmin, pos->lon);
      lonmax = wxMax(lonmax, pos->lon);
      pos = pos->next;
    } while (pos != (*it)->skippoints->point);
  }
}

void RouteMapOverlay::RequestGrib(wxDateTime time) {
  Json::Value v;
  time = time.FromUTC();
  v["Day"] = time.GetDay();
  v["Month"] = time.GetMonth();
  v["Year"] = time.GetYear();
  v["Hour"] = time.GetHour();
  v["Minute"] = time.GetMinute();
  v["Second"] = time.GetSecond();

  Json::FastWriter w;

  SendPluginMessage("GRIB_TIMELINE_RECORD_REQUEST", w.write(v));

  Lock();
  m_bNeedsGrib = false;
  Unlock();
}

std::list<PlotData>& RouteMapOverlay::GetPlotData(bool cursor_route) {
  std::list<PlotData>& plotdata =
      cursor_route ? last_cursor_plotdata : last_destination_plotdata;
  if (!cursor_route && clear_destination_plotdata) {
    clear_destination_plotdata = false;
    plotdata.clear();
  }
  if (plotdata.empty()) {
    Position* next =
        cursor_route ? last_cursor_position : last_destination_position;

    if (!next) return plotdata;

    Position* pos = next->parent;

    RouteMapConfiguration configuration = GetConfiguration();
    Lock();
    IsoChronList::iterator it = origin.begin(), itp;

    for (Position* p = pos; p; p = p->parent)
      if (++it == origin.end()) {
        Unlock();
        return plotdata;
      }
    it--;

    while (pos) {
      itp = it;
      itp--;

      configuration.grib = (*it)->m_Grib;
      configuration.time = (*it)->time;
      // printf("grib time %p %d\n", configuration.grib, configuration.time);

      configuration.UsedDeltaTime = (*it)->delta;
      PlotData data;

      double dt = configuration.UsedDeltaTime;
      data.time = (*it)->time;

      if (pos->GetPlotData(next, dt, configuration, data))
        plotdata.push_front(data);

      it = itp;
      next = pos;
      pos = pos->parent;
    }

    Unlock();
  }
  return plotdata;
}

double RouteMapOverlay::RouteInfo(enum RouteInfoType type, bool cursor_route) {
  std::list<PlotData>& plotdata = GetPlotData(cursor_route);

  double total = 0, count = 0, lat0 = 0, lon0 = 0;
  int comfort = 0, current_comfort = 0;
  for (std::list<PlotData>::iterator it = plotdata.begin();
       it != plotdata.end(); it++) {
    switch (type) {
      case DISTANCE: {
        if (it != plotdata.begin())
          total += DistGreatCircle_Plugin(lat0, lon0, it->lat, it->lon);

        lat0 = it->lat;
        lon0 = it->lon;
      } break;
      case AVGSPEED:
        total += it->stw;
        break;
      case MAXSPEED:
        if (total < it->stw) total = it->stw;
        break;
      case AVGSPEEDGROUND:
        total += it->sog;
        break;
      case MAXSPEEDGROUND:
        if (total < it->sog) total = it->sog;
        break;
      case AVGWIND:
        total += it->twsOverWater;
        break;
      case MAXWIND:
        if (total < it->twsOverWater) total = it->twsOverWater;
        break;
      case MAXWINDGUST:
        if (total < it->VW_GUST) total = it->VW_GUST;
        break;
      case AVGCURRENT:
        total += it->currentSpeed;
        break;
      case MAXCURRENT:
        if (total < it->currentSpeed) total = it->currentSpeed;
        break;
      case AVGSWELL:
        total += it->WVHT;
        break;
      case MAXSWELL:
        if (total < it->WVHT) total = it->WVHT;
        break;
      case PERCENTAGE_UPWIND:
        if (fabs(heading_resolve(it->ctw - it->twdOverWater)) < 90) total++;
        break;
      case PORT_STARBOARD:
        if (heading_resolve(it->ctw - it->twdOverWater) > 0) total++;
        break;
      // CUSTOMIZATION
      // Comfort on route
      case COMFORT:
        current_comfort = sailingConditionLevel(*it);
        if (current_comfort > comfort) comfort = current_comfort;
        break;
      default:
        break;
    }
    count++;
  }

  /* fixup data */
  switch (type) {
    case TACKS:
      return plotdata.size() ? plotdata.back().tacks : 0;
    case JIBES:
      return plotdata.size() ? plotdata.back().jibes : 0;
    case DISTANCE:
      if (total == 0)
        total = NAN;
      else if (Finished()) {
        RouteMapConfiguration configuration = GetConfiguration();
        total += DistGreatCircle_Plugin(lat0, lon0, configuration.EndLat,
                                        configuration.EndLon);
      }
      return total;
    case COMFORT:
      return comfort;
    case PERCENTAGE_UPWIND:
    case PORT_STARBOARD:
      total *= 100.0;
    case AVGSPEED:
    case AVGSPEEDGROUND:
    case AVGWIND:
    case AVGCURRENT:
    case AVGSWELL:
      total /= count;
    default:
      break;
  }
  return total;
}

/* how many cyclone tracks did we cross? which month? */
int RouteMapOverlay::Cyclones(int* months) {
  if (!RouteMap::ClimatologyCycloneTrackCrossings) return -1;

  int days = 30;  // search for 30 day range
  int cyclones = 0;

  Lock();
  wxDateTime ptime = m_EndTime;
  IsoChronList::iterator it = origin.end();

  for (Position* p = destination_position; p && p->parent; p = p->parent) {
    if (RouteMap::ClimatologyCycloneTrackCrossings(
            p->parent->lat, p->parent->lon, p->lat, p->lon, ptime, days)) {
      if (months) months[ptime.GetMonth()]++;
      cyclones++;
    }

    it--;
    ptime = (*it)->time;
  }

  Unlock();
  return cyclones;
}

void RouteMapOverlay::Clear() {
  RouteMap::Clear();
  last_cursor_position = nullptr;
  last_destination_position = nullptr;
  clear_destination_plotdata = false;
  // clear_cursor_plotdata = false;
  last_cursor_plotdata.clear();
  last_destination_plotdata.clear();
  m_UpdateOverlay = true;
}

void RouteMapOverlay::UpdateCursorPosition() {
  // only called in main thread, no race
  Position* last_last_cursor_position = last_cursor_position;
  last_cursor_position =
      ClosestPosition(last_cursor_lat, last_cursor_lon, &m_cursor_time);
  if (last_last_cursor_position != last_cursor_position)
    last_cursor_plotdata.clear();
}

bool RouteMapOverlay::SetCursorLatLon(double lat, double lon) {
  Position* p = last_cursor_position;
  last_cursor_lat = lat;
  last_cursor_lon = lon;

  UpdateCursorPosition();
  return p != last_cursor_position;
}

bool RouteMapOverlay::Updated() {
  bool updated = m_bUpdated;
  m_bUpdated = false;
  return updated;
}

void RouteMapOverlay::UpdateDestination() {
  RouteMapConfiguration configuration = GetConfiguration();
  Position* last_last_destination_position = last_destination_position;
  bool done = ReachedDestination();
  if (done) {
    Lock();
    delete destination_position;
    destination_position = 0;
    /* this doesn't happen often, so can be slow.. for each position in the last
       isochron, we try to propagate to the destination */
    IsoChronList::iterator iit = origin.end();
    iit--;
    iit--; /* second from last isochron */
    IsoChron* isochron = *iit;
    double mindt = INFINITY;
    Position* endp;
    double minH;
    bool mintacked;
    bool minjibes;
    bool minsail_plan_changed;
    DataMask mindata_mask;

    for (IsoRouteList::iterator it = isochron->routes.begin();
         it != isochron->routes.end(); ++it) {
      configuration.grib = isochron->m_Grib;
      configuration.grib_is_data_deficient = isochron->m_Grib_is_data_deficient;

      configuration.time = isochron->time;
      configuration.UsedDeltaTime = isochron->delta;
      (*it)->PropagateToEnd(configuration, mindt, endp, minH, mintacked,
                            minjibes, minsail_plan_changed, mindata_mask);
    }
    Unlock();

    if (std::isinf(mindt)) {
      // destination is between two isochrons
      // but propagate can't reach it (land or boundaries in the way).
      // Use an upper bound time for EndTime, not defined times are too much
      // trouble later.
      m_EndTime = isochron->time + wxTimeSpan(0, 0, isochron->delta);
      last_destination_position =
          ClosestPosition(configuration.EndLat, configuration.EndLon);
    } else {
      destination_position = new Position(
          configuration.EndLat, configuration.EndLon, endp, minH, NAN,
          endp->polar, endp->tacks + mintacked, endp->jibes + minjibes,
          endp->sail_plan_changes + minsail_plan_changed, mindata_mask);

      m_EndTime = isochron->time + wxTimeSpan::Milliseconds(1000 * mindt);
      isochron->delta = mindt;
      last_destination_position = destination_position;
    }
  } else {
    last_destination_position =
        ClosestPosition(configuration.EndLat, configuration.EndLon);

    m_EndTime = wxDateTime();  // invalid
  }

  if (last_last_destination_position != last_destination_position) {
    // we can't clear because we are inside a worker thread
    // and there's a race with GetPlotData
    clear_destination_plotdata = true;
  }

  m_bUpdated = true;
  m_UpdateOverlay = true;
}

// CUSTOMIZATION

Position* RouteMapOverlay::getClosestRoutePositionFromCursor(
    double cursorLat, double cursorLon, PlotData& posData) {
  /* Method to find the closest calculated position of the boat on
   * the weather route based on the cursor position
   */

  double dist = INFINITY;
  std::list<PlotData> plot = GetPlotData(false);
  bool found = false;
  posData.time = wxInvalidDateTime;
  for (const auto& it : plot) {
    // Calculate distance
    // Almost like a plan (x,y) because of small distance -- is that correct?
    double tempDist =
        sqrt(pow(cursorLat - it.lat, 2) + pow(cursorLon - it.lon, 2));
    if (tempDist < dist) {
      posData = it;
      dist = tempDist;
      found = true;
    }
  }
  if (!found) return nullptr;

  // Get full position
  Position* pos = last_destination_position;
  for (Position* p = pos; p && p->parent; p = p->parent) {
    if (p->lat == posData.lat && p->lon == posData.lon) {
      return p;
    }
  }
  return nullptr;
}
