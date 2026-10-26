// Copyright 2026 cad-less-robotics contributors

#include "robot_controller/waypoint_prep.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

TEST(WaypointPrep, ApproachLiftsOppositeTheToolZAxis) {
  robot_controller::Pose start{0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
  robot_controller::Pose next = start;
  next.x = 0.01;
  const auto output = robot_controller::withApproachRetreat({start, next}, 0.02, 0.02);
  ASSERT_EQ(output.size(), 4u);
  EXPECT_NEAR(output.front().z, 0.02, 1e-9);
  EXPECT_NEAR(output[1].z, 0.0, 1e-9);
  EXPECT_NEAR(output.back().z, 0.02, 1e-9);
}

TEST(WaypointPrep, LimitKeepsEndpointsAndMidpoint) {
  std::vector<robot_controller::Pose> poses;
  for (int index = 0; index < 5; ++index) {
    robot_controller::Pose pose;
    pose.x = 0.01 * index;
    poses.push_back(pose);
  }
  const auto output = robot_controller::limitWaypointCount(poses, 3);
  ASSERT_EQ(output.size(), 3u);
  EXPECT_NEAR(output.front().x, 0.0, 1e-9);
  EXPECT_NEAR(output[1].x, 0.02, 1e-9);
  EXPECT_NEAR(output.back().x, 0.04, 1e-9);
}

TEST(WaypointPrep, HemisphereAlignmentMakesAdjacentDotPositive) {
  std::vector<robot_controller::Pose> poses(2);
  poses[0].qw = 1.0;
  poses[1].qw = -1.0;
  robot_controller::alignQuaternionHemisphere(poses);
  const double dot = (poses[0].qx * poses[1].qx) + (poses[0].qy * poses[1].qy) +
                     (poses[0].qz * poses[1].qz) + (poses[0].qw * poses[1].qw);
  EXPECT_GT(dot, 0.0);
}

TEST(WaypointPrep, DropsPointsCloserThanSeparation) {
  robot_controller::Pose a;
  robot_controller::Pose b;
  b.x = 0.0001;
  robot_controller::Pose c;
  c.x = 0.01;
  const auto output = robot_controller::dropClosePoses({a, b, c}, 0.0005);
  ASSERT_EQ(output.size(), 2u);
  EXPECT_NEAR(output.back().x, 0.01, 1e-12);
}

TEST(WaypointPrep, RejectsAPoseBeyondReach) {
  robot_controller::Pose near;
  robot_controller::Pose far;
  far.x = 1.2;
  EXPECT_NE(robot_controller::pathError({near, far}, 1.0).find("도달"), std::string::npos);
}

TEST(WaypointPrep, NormalizesAScaledQuaternion) {
  std::vector<robot_controller::Pose> poses(1);
  poses[0].qw = 2.0;
  ASSERT_TRUE(robot_controller::normalizeOrientations(poses));
  EXPECT_NEAR(poses[0].qw, 1.0, 1e-12);
}

TEST(WaypointPrep, RejectsNegativeApproach) {
  robot_controller::Pose pose;
  EXPECT_THROW(robot_controller::withApproachRetreat({pose}, -0.01, 0.0), std::invalid_argument);
}
