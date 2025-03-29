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

#include <gtest/gtest.h>
#include <Polar.h>

wxString
  testDataDir = TESTDATADIR,
  testPolarFileRelativePath = "polars/Hallberg-Rassy_40_test.pol",
  testPolarFileName = testDataDir + "/" + testPolarFileRelativePath,
  // testPolarFileName = "/Users/quinton/src/OpenCPN-clean/weather_routing_pi-Richard/weather_routing_pi/build/data/polars/Hallberg-Rassy_40.pol",
  testFileOpenMessage("");
 
  // Demonstrate some basic assertions.
TEST(PolarTests, AssertionsBasic) {
  // Expect two strings not to be equal.
  EXPECT_STRNE("hello", "world");
  // Expect equality.
  EXPECT_EQ(7 * 6, 42);
}

TEST(PolarTests, ConstructorBasic) {
  Polar p;
  EXPECT_EQ(p.FileName, "");
} 

TEST(PolarTests, OpenFailed) {
  Polar p;
  wxString filename = "test.xml", message = "";
  bool success = p.Open(filename, message);
  EXPECT_EQ(success, false);
}

TEST(PolarTests, OpenSuccess) {
  Polar p;
  std::cout << "TESTDATADIR: " << testDataDir << std::endl;
  std::cout << "testPolarFileRelativePath: " << testPolarFileRelativePath << std::endl;
  std::cout << "testPolarFileName: " << testPolarFileName << std::endl;

  bool success = p.Open(testPolarFileName, testFileOpenMessage);
  EXPECT_EQ(testFileOpenMessage.ToStdString(), std::string(""));
  EXPECT_EQ(success, true);
}

TEST(PolarTests, ClosestVWiBasic) {
  Polar p;
  bool success = p.Open(testPolarFileName, testFileOpenMessage);
  EXPECT_EQ(success, true);
  int VW1i, VW2i;
  p.ClosestVWi(10, VW1i, VW2i);
  EXPECT_EQ(VW1i, 4);
  EXPECT_EQ(VW2i, 5);
}

TEST(PolarTests, SpeedBasic) {
  Polar p;
  PolarSpeedStatus status;
  p.Open(testPolarFileName, testFileOpenMessage);
  double speed = p.Speed(10, 10, &status, false);
  EXPECT_NEAR(speed, 1.3, 1e-6);
}

TEST(PolarTests, SpeedAtApparentWindDirectionBasic) {
  Polar p;
  p.Open(testPolarFileName, testFileOpenMessage);
  double W;
  double speed = p.SpeedAtApparentWindDirection(10, 10, &W);
  EXPECT_NEAR(speed, 1.473, 1e-3);
  EXPECT_NEAR(W, 11.444, 1e-3);
}

TEST(PolarTests, SpeedAtApparentWindBasic) {
  Polar p;
  p.Open(testPolarFileName, testFileOpenMessage);
  double TWA;
  double speed = p.SpeedAtApparentWind(90, 10, &TWA);
  EXPECT_NEAR(speed, 7.669, 1e-3);
  EXPECT_NEAR(TWA, 127.498, 1e-3);
}

TEST(PolarTests, GetVMGTrueWindBasic) {
  Polar p;
  p.Open(testPolarFileName, testFileOpenMessage);
  SailingVMG vmg = p.GetVMGTrueWind(10);
  EXPECT_NEAR(vmg.values[SailingVMG::STARBOARD_UPWIND], 44.998, 1e-3);
  EXPECT_NEAR(vmg.values[SailingVMG::PORT_UPWIND], 315.002, 1e-3);
  EXPECT_NEAR(vmg.values[SailingVMG::STARBOARD_DOWNWIND], 152.498, 1e-3);
  EXPECT_NEAR(vmg.values[SailingVMG::PORT_DOWNWIND], 207.501, 1e-3);
}

TEST(PolarTests, GetVMGApparentWindBasic) {
  Polar p;
  p.Open(testPolarFileName, testFileOpenMessage);
  SailingVMG vmg = p.GetVMGApparentWind(10);
  EXPECT_NEAR(vmg.values[SailingVMG::STARBOARD_UPWIND], 45.873, 1e-3);
  EXPECT_NEAR(vmg.values[SailingVMG::PORT_UPWIND], 314.127, 1e-3);
  EXPECT_NEAR(vmg.values[SailingVMG::STARBOARD_DOWNWIND], 169.359, 1e-3);
  EXPECT_NEAR(vmg.values[SailingVMG::PORT_DOWNWIND], 190.640, 1e-3);
}

TEST(PolarTests, TrueWindSpeedBasic) {
  Polar p;
  p.Open(testPolarFileName, testFileOpenMessage);
  double speed = p.TrueWindSpeed(5, 90, 80);
  EXPECT_NEAR(speed, 4.428, 1e-3);
}

TEST(PolarTests, InterpolateSpeedsBasic) {
  Polar p;
  p.Open(testPolarFileName, testFileOpenMessage);
  bool success = p.InterpolateSpeeds();
  EXPECT_EQ(success, false); // @todo: The call fails. Figure out why, and fix this test.
}

TEST(PolarTests, UpdateSpeedsBasic) {
  Polar p;
  p.Open(testPolarFileName, testFileOpenMessage);
  p.UpdateSpeeds(); // @todo: The call succeeded, but did it do the right thing?  Test that.
}

TEST(PolarTests, UpdateDegreeStepLookupBasic) {
  Polar p;
  p.Open(testPolarFileName, testFileOpenMessage);
  p.UpdateDegreeStepLookup(); // @todo: The call succeeded, but did it do the right thing?  Test that.
}

TEST(PolarTests, InsideCrossOverContourBasic) {
  Polar p;
  p.Open(testPolarFileName, testFileOpenMessage);
  bool isInside = p.InsideCrossOverContour(10, 10, true);
  EXPECT_EQ(isInside, false); // @todo: I think we need to load more polars to test this properly.
}

TEST(PolarTests, GenerateBasic) {
  Polar p;
  p.Open(testPolarFileName, testFileOpenMessage);
  p.Generate(std::list<PolarMeasurement>()); // @todo: The call succeeded, but did it do the right thing?  Test that.
}

TEST(PolarTests, AddDegreeStepBasic) {
  Polar p;
  p.Open(testPolarFileName, testFileOpenMessage);
  p.AddDegreeStep(10); // @todo: The call succeeded, but did it do the right thing?  Test that.
}

TEST(PolarTests, RemoveDegreeStepBasic) {
  Polar p;
  p.Open(testPolarFileName, testFileOpenMessage);
  p.RemoveDegreeStep(10); // @todo: The call succeeded, but did it do the right thing?  Test that.
}

TEST(PolarTests, OptimizeTackingSpeedBasic) {
  Polar p;
  p.Open(testPolarFileName, testFileOpenMessage);
  // p.OptimizeTackingSpeed(); // @todo: This method is not defined yet.
}

TEST(PolarTests, AddWindSpeedBasic) {
  Polar p;
  p.Open(testPolarFileName, testFileOpenMessage);
  p.AddWindSpeed(10); // @todo: The call succeeded, but did it do the right thing?  Test that.
}

TEST(PolarTests, RemoveWindSpeedBasic) {
  Polar p;
  p.Open(testPolarFileName, testFileOpenMessage);
  p.RemoveWindSpeed(10); // @todo: The call succeeded, but did it do the right thing?  Test that.
}

TEST(PolarTests, VelocityApparentWindBasic) {
  double speed = Polar::VelocityApparentWind(10, 10, 10);
  EXPECT_NEAR(speed, 19.923, 1e-3);
}

TEST(PolarTests, VelocityTrueWindBasic) {
  // Dead upwind
  double speed = Polar::VelocityTrueWind(5, 10, 0);
  EXPECT_NEAR(speed, -5.000, 1e-3);
  // Beating
  speed = Polar::VelocityTrueWind(8, 5, 45);
  EXPECT_NEAR(speed, 3.640, 1e-3);
  // Beam reach
  speed = Polar::VelocityTrueWind(6, 5, 90);
  EXPECT_NEAR(speed, 3.316, 1e-3);
  // Broad reach
  speed = Polar::VelocityTrueWind(5, 5, 120);
  EXPECT_NEAR(speed, 5.000, 1e-3);
  // Dead downwind
  speed = Polar::VelocityTrueWind(-5, 10, 180);
  EXPECT_NEAR(speed, 15.000, 1e-3);
}
 
TEST(PolarTests, DirectionApparentWindBasic) {
  // Returns 0 if aws is 0 (apparent wind direction undefined)
  double direction = Polar::DirectionApparentWind(0, 10, 10, 10);
  EXPECT_NEAR(direction, 0.000, 1e-3);

  // Returns W if stw is 0 (no boat motion, apparent = true wind)
  direction = Polar::DirectionApparentWind(10, 0, 10, 10);
  EXPECT_NEAR(direction, 10.000, 1e-3);

  // Otherwise returns calculated angle accounting for both vectors
  direction = Polar::DirectionApparentWind(10, 5, 90, 10);
  EXPECT_NEAR(direction, 75.522, 1e-3);
}

TEST(PolarTests, DirectionApparentWind2Basic) {
  double direction = Polar::DirectionApparentWind(5, 90, 10);
  EXPECT_NEAR(direction, 63.434, 1e-3);
}
