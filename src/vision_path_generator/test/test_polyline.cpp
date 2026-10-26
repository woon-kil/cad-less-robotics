// Copyright 2026 cad-less-robotics contributors

#include "vision_path_generator/polyline.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>
#include <vector>

namespace {

vision_path_generator::Vec3 cross(const vision_path_generator::Vec3& a, const vision_path_generator::Vec3& b) {
  return vision_path_generator::Vec3{
      (a.y * b.z) - (a.z * b.y), (a.z * b.x) - (a.x * b.z), (a.x * b.y) - (a.y * b.x)};
}

}  // namespace

TEST(Resample, ExactLengthKeepsTwoMillimeterSpacingAndEndpoint) {
  const std::vector<vision_path_generator::Vec3> line{{0.0, 0.0, 0.0}, {0.1, 0.0, 0.0}};
  const auto output = vision_path_generator::resamplePolyline(line, 0.002);
  ASSERT_EQ(output.size(), 51u);
  EXPECT_NEAR(output.front().x, 0.0, 1e-9);
  EXPECT_NEAR(output[1].x, 0.002, 1e-9);
  EXPECT_NEAR(output.back().x, 0.1, 1e-9);
}

TEST(Resample, ShortTailStillReachesEndpoint) {
  const std::vector<vision_path_generator::Vec3> line{{0.0, 0.0, 0.0}, {0.005, 0.0, 0.0}};
  const auto output = vision_path_generator::resamplePolyline(line, 0.002);
  ASSERT_EQ(output.size(), 4u);
  EXPECT_NEAR(output[1].x, 0.002, 1e-9);
  EXPECT_NEAR(output[2].x, 0.004, 1e-9);
  EXPECT_NEAR(output.back().x, 0.005, 1e-9);
}

TEST(Resample, RejectsNonPositiveSpacing) {
  const std::vector<vision_path_generator::Vec3> line{{0.0, 0.0, 0.0}, {0.01, 0.0, 0.0}};
  EXPECT_THROW(vision_path_generator::resamplePolyline(line, 0.0), std::invalid_argument);
}

TEST(Chain, KeepsTheLongerSeparatedCluster) {
  std::vector<vision_path_generator::Vec3> points;
  for (int index = 0; index < 3; ++index) {
    points.push_back({0.002 * index, 0.0, 0.0});
  }
  for (int index = 0; index < 5; ++index) {
    points.push_back({1.0 + (0.002 * index), 0.0, 0.0});
  }
  const auto chain = vision_path_generator::longestNearestNeighborChain(points, 0.02);
  ASSERT_EQ(chain.size(), 5u);
  for (const auto& point : chain) {
    EXPECT_GT(point.x, 0.5);
  }
}

TEST(Chain, OrdersAShuffledLineFromAnEndpoint) {
  const std::vector<vision_path_generator::Vec3> shuffled{
      {0.008, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.004, 0.0, 0.0}, {0.002, 0.0, 0.0}, {0.006, 0.0, 0.0}};
  const auto chain = vision_path_generator::longestNearestNeighborChain(shuffled, 0.02);
  ASSERT_EQ(chain.size(), 5u);
  const bool increasing = chain.front().x < chain.back().x;
  for (std::size_t index = 1; index < chain.size(); ++index) {
    if (increasing) {
      EXPECT_LT(chain[index - 1].x, chain[index].x);
    } else {
      EXPECT_GT(chain[index - 1].x, chain[index].x);
    }
  }
}

TEST(Smooth, KeepsEndpointsAndAveragesTheMiddle) {
  const std::vector<vision_path_generator::Vec3> input{{0.0, 0.0, 0.0}, {1.0, 1.0, 0.0}, {2.0, 0.0, 0.0}};
  const auto output = vision_path_generator::smoothOpenPolyline(input, 1);
  ASSERT_EQ(output.size(), 3u);
  EXPECT_NEAR(output.front().x, 0.0, 1e-12);
  EXPECT_NEAR(output.back().x, 2.0, 1e-12);
  EXPECT_NEAR(output[1].x, 1.0, 1e-12);
  EXPECT_NEAR(output[1].y, 1.0 / 3.0, 1e-12);
}

TEST(Orientation, ToolAxesFollowNormalAndTangent) {
  const auto orientation = vision_path_generator::orientationFromTangentAndNormal(
      vision_path_generator::Vec3{1.0, 0.0, 0.0}, vision_path_generator::Vec3{0.0, 0.0, 1.0});
  const auto tool_x = vision_path_generator::rotateVector(orientation, {1.0, 0.0, 0.0});
  const auto tool_y = vision_path_generator::rotateVector(orientation, {0.0, 1.0, 0.0});
  const auto tool_z = vision_path_generator::rotateVector(orientation, {0.0, 0.0, 1.0});
  EXPECT_NEAR(tool_x.x, 1.0, 1e-9);
  EXPECT_NEAR(tool_z.z, -1.0, 1e-9);
  const auto handed = cross(tool_x, tool_y);
  EXPECT_NEAR(handed.x, tool_z.x, 1e-9);
  EXPECT_NEAR(handed.y, tool_z.y, 1e-9);
  EXPECT_NEAR(handed.z, tool_z.z, 1e-9);
}

TEST(NormalOrientation, FlipsANormalThatPointsAwayFromTheViewpoint) {
  const auto oriented = vision_path_generator::orientNormalTowardsViewpoint(
      {0.0, 0.0, 1.0}, {0.0, 0.0, 1.0}, {0.0, 0.0, 0.0});
  EXPECT_NEAR(oriented.x, 0.0, 1e-12);
  EXPECT_NEAR(oriented.y, 0.0, 1e-12);
  EXPECT_NEAR(oriented.z, -1.0, 1e-12);
}

TEST(NormalOrientation, KeepsANormalThatAlreadyFacesTheViewpoint) {
  const auto oriented = vision_path_generator::orientNormalTowardsViewpoint(
      {0.0, 0.0, 1.0}, {0.0, 0.0, -2.0}, {0.0, 0.0, 0.0});
  EXPECT_NEAR(oriented.x, 0.0, 1e-12);
  EXPECT_NEAR(oriented.y, 0.0, 1e-12);
  EXPECT_NEAR(oriented.z, -1.0, 1e-12);
}

TEST(NormalOrientation, RejectsANonFiniteOrZeroNormal) {
  const auto invalid = vision_path_generator::orientNormalTowardsViewpoint(
      {0.0, 0.0, 1.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0});
  EXPECT_NEAR(invalid.x, 0.0, 1e-12);
  EXPECT_NEAR(invalid.y, 0.0, 1e-12);
  EXPECT_NEAR(invalid.z, 0.0, 1e-12);
}

TEST(NormalOrientation, DoesNotPropagateAcrossACameraFacingCorner) {
  const vision_path_generator::Vec3 viewpoint{0.0, 0.0, 0.0};
  const vision_path_generator::Vec3 point{0.2, 0.0, 1.0};
  const auto left = vision_path_generator::orientNormalTowardsViewpoint(
      point, {-1.0, 0.0, -0.3}, viewpoint);
  const auto right = vision_path_generator::orientNormalTowardsViewpoint(
      point, {1.0, 0.0, -0.3}, viewpoint);
  EXPECT_LT(vision_path_generator::dot(left, point), 0.0);
  EXPECT_LT(vision_path_generator::dot(right, point), 0.0);
  EXPECT_LT(vision_path_generator::dot(left, right), 0.0);
}

TEST(NormalReliability, SeparatesTheNormalAxisByAtLeastAFactorOfTwo) {
  EXPECT_NEAR(vision_path_generator::normalAxisSeparation(1.0, 2.0), 0.5, 1e-12);
  EXPECT_NEAR(vision_path_generator::kMinNormalAxisSeparation, 0.5, 1e-12);
  EXPECT_GT(
      vision_path_generator::normalAxisSeparation(0.2, 1.0),
      vision_path_generator::kMinNormalAxisSeparation);
  EXPECT_LT(vision_path_generator::normalAxisSeparation(0.8, 1.0), vision_path_generator::kMinNormalAxisSeparation);
}

TEST(NormalRecovery, ReplacesAnIsolatedNormalWhenBothSupportsAgree) {
  const auto recovered = vision_path_generator::recoverNormalFromSupports(
      {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0}, {0.0, 0.0, -1.0}, {0.0, 0.2, -1.0}, true, true, {0.0, 0.0, 0.0});
  EXPECT_EQ(recovered.recovery, vision_path_generator::NormalRecovery::Recovered);
  EXPECT_NEAR(recovered.normal.x, 0.0, 1e-9);
  EXPECT_LT(recovered.normal.z, 0.0);
  EXPECT_GT(vision_path_generator::dot(recovered.normal, {0.0, 0.0, -1.0}), 0.0);
}

TEST(NormalRecovery, DoesNotBlendSupportsAcrossASharpEdge) {
  const vision_path_generator::Vec3 original{0.0, 1.0, 0.0};
  const auto recovered = vision_path_generator::recoverNormalFromSupports(
      {0.0, 0.0, 1.0}, original, {-1.0, 0.0, -0.2}, {1.0, 0.0, -0.2}, true, true, {0.0, 0.0, 0.0});
  EXPECT_EQ(recovered.recovery, vision_path_generator::NormalRecovery::UnrecoverableEdge);
  EXPECT_NEAR(recovered.normal.x, original.x, 1e-12);
  EXPECT_NEAR(recovered.normal.y, original.y, 1e-12);
  EXPECT_NEAR(recovered.normal.z, original.z, 1e-12);
}

TEST(NormalDiscontinuity, KeepsACoherentCameraFacingCorner) {
  const auto region_a = vision_path_generator::normalized({-0.9, 0.0, -0.2});
  const auto region_b = vision_path_generator::normalized({0.9, 0.0, -0.2});
  const auto kind = vision_path_generator::classifyNormalDiscontinuity(
      region_a, region_a, region_b, region_b, {0.001, 0.0, 1.0}, {-0.001, 0.0, 1.0}, 0.002, 0.02, true,
      true);
  EXPECT_EQ(kind, vision_path_generator::NormalDiscontinuity::Geometric);
}

TEST(NormalDiscontinuity, MarksASignFlipThatStillFacesTheCamera) {
  const vision_path_generator::Vec3 away{0.0, 0.0, 1.0};
  const vision_path_generator::Vec3 toward{0.0, 0.0, -1.0};
  const auto kind = vision_path_generator::classifyNormalDiscontinuity(
      toward, away, toward, toward, {0.0, 0.0, 1.0}, {0.2, 0.0, 1.0}, 0.002, 0.02, true, true);
  EXPECT_EQ(kind, vision_path_generator::NormalDiscontinuity::SignAmbiguity);
}

TEST(Orientation, FlipsQuaternionToTheSameHemisphere) {
  std::vector<vision_path_generator::Quat> orientations{{0.0, 0.0, 0.0, 1.0}, {0.0, 0.0, 0.0, -1.0}};
  vision_path_generator::alignQuaternionHemisphere(orientations);
  const double dot = (orientations[0].x * orientations[1].x) + (orientations[0].y * orientations[1].y) +
                     (orientations[0].z * orientations[1].z) + (orientations[0].w * orientations[1].w);
  EXPECT_GT(dot, 0.0);
}
