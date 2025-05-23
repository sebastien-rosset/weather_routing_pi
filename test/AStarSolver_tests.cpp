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

#include <wx/wx.h>
#include <gtest/gtest.h>
#include "RouteMap.h"

/**
 * Simple concrete implementation of RouteMap for testing.
 * Implements the pure virtual methods with minimal functionality.
 */
class TestableRouteMap : public RouteMap {
public:
    TestableRouteMap() : m_mutex() {}
    
protected:
    virtual void Lock() override { m_mutex.Lock(); }
    virtual void Unlock() override { m_mutex.Unlock(); }
    virtual bool TestAbort() override { return false; }
    
private:
    wxMutex m_mutex;
};

/**
 * Test suite for A* solver implementation.
 * These tests verify basic functionality of the A* weather routing solver.
 */
class AStarSolverTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Setup basic test configuration
        config.StartLat = 37.7749;   // San Francisco
        config.StartLon = -122.4194;
        config.EndLat = 37.8044;     // Alcatraz Island (short distance for testing)
        config.EndLon = -122.4078;
        config.StartTime = wxDateTime::Now();
        config.DeltaTime = 300;      // 5 minutes
        config.UseGrib = false;      // Simplified test without GRIB
        config.solver_type = RouteMapConfiguration::SOLVER_ASTAR;
        
        // Set reasonable defaults for other parameters
        config.MaxTrueWindKnots = 50.0;
        config.MaxApparentWindKnots = 50.0;
        config.MaxLatitude = 90.0;
        config.TackingTime = 0.0;
        config.JibingTime = 0.0;
        config.SailPlanChangeTime = 0.0;
    }

    RouteMapConfiguration config;
};

/**
 * Test basic A* solver initialization and configuration.
 */
TEST_F(AStarSolverTest, BasicInitialization) {
    TestableRouteMap route_map;
    route_map.SetConfiguration(config);
    
    // Verify configuration was set correctly
    EXPECT_TRUE(route_map.Valid());
    
    RouteMapConfiguration retrieved_config = route_map.GetConfiguration();
    EXPECT_EQ(retrieved_config.solver_type, RouteMapConfiguration::SOLVER_ASTAR);
    EXPECT_NEAR(retrieved_config.StartLat, config.StartLat, 0.0001);
    EXPECT_NEAR(retrieved_config.StartLon, config.StartLon, 0.0001);
}

/**
 * Test A* solver can find a basic route between two close points.
 */
TEST_F(AStarSolverTest, BasicRouteFinds) {
    TestableRouteMap route_map;
    route_map.SetConfiguration(config);
    
    // Run A* solver
    bool found_route = route_map.RunAStarSolver();
    
    // Should find a route for this short distance
    EXPECT_TRUE(found_route);
    EXPECT_TRUE(route_map.AStarFinished());
    EXPECT_TRUE(route_map.AStarFoundRoute());
    
    // Check that we have a solution path
    std::list<AStarNode*> path = route_map.GetAStarSolutionPath();
    EXPECT_GT(path.size(), 0);
}

/**
 * Test A* node creation and basic properties.
 */
TEST_F(AStarSolverTest, NodeCreation) {
    wxDateTime test_time = wxDateTime::Now();
    AStarNode node(37.7749, -122.4194, test_time, 90.0);
    
    EXPECT_NEAR(node.lat, 37.7749, 0.0001);
    EXPECT_NEAR(node.lon, -122.4194, 0.0001);
    EXPECT_EQ(node.time, test_time);
    EXPECT_NEAR(node.heading, 90.0, 0.1);
    EXPECT_EQ(node.parent, nullptr);
    EXPECT_FALSE(node.in_open_set);
    EXPECT_FALSE(node.in_closed_set);
}

/**
 * Test distance calculation between A* nodes.
 */
TEST_F(AStarSolverTest, NodeDistanceCalculation) {
    wxDateTime test_time = wxDateTime::Now();
    AStarNode node1(37.7749, -122.4194, test_time, 0.0);  // San Francisco
    AStarNode node2(37.8044, -122.4078, test_time, 0.0);  // Alcatraz
    
    double distance = node1.DistanceTo(&node2);
    
    // Distance should be approximately 1.2 nautical miles
    EXPECT_GT(distance, 1.0);
    EXPECT_LT(distance, 2.0);
}

/**
 * Test path reconstruction functionality.
 */
TEST_F(AStarSolverTest, PathReconstruction) {
    wxDateTime test_time = wxDateTime::Now();
    
    // Create a simple 3-node path
    AStarNode start(37.7749, -122.4194, test_time, 0.0);
    AStarNode middle(37.7850, -122.4150, test_time, 45.0, &start);
    AStarNode end(37.8044, -122.4078, test_time, 90.0, &middle);
    
    std::list<AStarNode*> path = end.ReconstructPath();
    
    EXPECT_EQ(path.size(), 3);
    EXPECT_EQ(path.front(), &start);
    EXPECT_EQ(path.back(), &end);
}

/**
 * Test that the solver test utility works.
 */
TEST_F(AStarSolverTest, SolverTestUtility) {
    wxString results = RouteMap::RunSolverTest();
    
    // Should contain test results
    EXPECT_TRUE(results.Contains("Solver Test Results"));
    EXPECT_TRUE(results.Contains("A*"));
}

/**
 * Test TestSolver method with A* option.
 */
TEST_F(AStarSolverTest, TestSolverMethod) {
    TestableRouteMap route_map;
    route_map.SetConfiguration(config);
    
    // Test A* solver via TestSolver method
    bool result = route_map.TestSolver(true);  // true = use A*
    
    EXPECT_TRUE(result);
    EXPECT_TRUE(route_map.AStarFinished());
}

/**
 * Test that invalid configuration is handled properly.
 */
TEST_F(AStarSolverTest, InvalidConfiguration) {
    RouteMapConfiguration invalid_config;
    // Don't set start/end positions - should be invalid
    
    TestableRouteMap route_map;
    route_map.SetConfiguration(invalid_config);
    
    EXPECT_FALSE(route_map.Valid());
    
    bool result = route_map.RunAStarSolver();
    EXPECT_FALSE(result);
}

/**
 * Performance test - ensure A* completes in reasonable time.
 */
TEST_F(AStarSolverTest, PerformanceTest) {
    TestableRouteMap route_map;
    route_map.SetConfiguration(config);
    
    auto start_time = std::chrono::high_resolution_clock::now();
    bool found_route = route_map.RunAStarSolver();
    auto end_time = std::chrono::high_resolution_clock::now();
    
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    // Should complete within 5 seconds for this short route
    EXPECT_LT(duration.count(), 5000);
    EXPECT_TRUE(found_route);
}
