// Copyright 2021 Apex.AI, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// \copyright Copyright 2021 Apex.AI, Inc.
/// All rights reserved.

#include <geometry_msgs/msg/point32.hpp>
#include <point_cloud_msg_wrapper/default_field_generators.hpp>
#include <point_cloud_msg_wrapper/field_generators.hpp>
#include <point_cloud_msg_wrapper/point_cloud_msg_wrapper.hpp>
#include <point_cloud_msg_wrapper/test_point_types.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <tuple>

namespace
{

LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(non_standard_test_field);


using point_cloud_msg_wrapper::PointCloud2View;
using point_cloud_msg_wrapper::PointCloud2Modifier;
using point_cloud_msg_wrapper::PointCloud2PartialView;
using point_cloud_msg_wrapper::PointCloud2PartialModifier;

using point_cloud_msg_wrapper::test::PointX;
using point_cloud_msg_wrapper::test::PointXYZI;
using point_cloud_msg_wrapper::test::ClassPointXY;
using point_cloud_msg_wrapper::test::PointXYWithVirtualDestructor;
using point_cloud_msg_wrapper::test::CustomAlignedPoint;
using point_cloud_msg_wrapper::test::GeometryPointXYZ;
using point_cloud_msg_wrapper::test::PointNotPresentInAllPointTypes;
using point_cloud_msg_wrapper::test::PointWithCustomField;
using point_cloud_msg_wrapper::test::create_point;

}  // namespace

template<typename T>
class PointCloudMsgWrapperTest : public testing::Test
{
public:
  using PointT = T;
};

using AllPointTypes = ::testing::Types<
  PointX,
  PointXYZI,
  ClassPointXY,
  PointXYWithVirtualDestructor,
  CustomAlignedPoint,
  GeometryPointXYZ>;
// cppcheck-suppress syntaxError - trailing comma is the only way to remove the compiler warning.
TYPED_TEST_CASE(PointCloudMsgWrapperTest, AllPointTypes, );


/// @test Test that for any of the different types of points we can read and write them into msg.
TYPED_TEST(PointCloudMsgWrapperTest, ReadingAndWritingGenericPoints)
{
  using Point = typename TestFixture::PointT;
  sensor_msgs::msg::PointCloud2 msg;

  // Cannot initialize a wrapper without resetting an empty message.
  EXPECT_THROW(PointCloud2View<Point>{msg}, std::runtime_error);
  EXPECT_THROW(PointCloud2Modifier<Point>{msg}, std::runtime_error);

  PointCloud2Modifier<Point> cloud_wrapper{msg, "some_frame_id"};
  EXPECT_EQ(msg.header.frame_id, "some_frame_id");

  const auto point = create_point<Point>();

  ASSERT_FALSE(msg.fields.empty());
  ASSERT_FALSE(msg.fields.front().name.empty());
  ASSERT_NO_THROW(cloud_wrapper.push_back(point));

  ASSERT_THROW(PointCloud2View<PointNotPresentInAllPointTypes>{msg}, std::runtime_error);
  ASSERT_THROW(PointCloud2Modifier<PointNotPresentInAllPointTypes>{msg}, std::runtime_error);

  ASSERT_TRUE(PointCloud2Modifier<Point>::can_be_created_from(msg));
  const auto initialized_wrapper = PointCloud2Modifier<Point>{msg};
  ASSERT_EQ(initialized_wrapper.size(), 1U);
  EXPECT_EQ(initialized_wrapper.at(0U), point);
  EXPECT_EQ(initialized_wrapper[0U], point);
  EXPECT_NO_THROW(cloud_wrapper.push_back(initialized_wrapper[0U]));
  EXPECT_EQ(initialized_wrapper.size(), 2U);

  const auto & const_cloud = msg;
  const PointCloud2View<Point> const_wrapper{const_cloud};
  ASSERT_EQ(const_wrapper.size(), 2U);
  EXPECT_EQ(const_wrapper.at(0U), point);
  EXPECT_EQ(const_wrapper[0U], point);
  EXPECT_EQ(const_wrapper.at(1U), point);
  EXPECT_EQ(const_wrapper[1U], point);

  // Test that we can iterate over the message.
  std::array<Point, 2U> points;
  std::size_t index{};
  for (const auto & p : cloud_wrapper) {
    points[index] = p;
    index++;
  }
  ASSERT_EQ(points.size(), cloud_wrapper.size());
  for (auto i = 0U; i < points.size(); ++i) {
    EXPECT_EQ(points[i], cloud_wrapper[i]);
  }
  EXPECT_EQ(points.front(), cloud_wrapper.front());
  EXPECT_EQ(points.back(), cloud_wrapper.back());

  cloud_wrapper.resize(3U);
  EXPECT_EQ(cloud_wrapper.size(), 3U);
  // Check that the untouched points stayed the same.
  for (auto i = 0U; i < points.size(); ++i) {
    EXPECT_EQ(points[i], cloud_wrapper[i]);
  }

  // Change values.
  for (auto & p : cloud_wrapper) {
    p = Point{};
  }
  for (const auto & p : cloud_wrapper) {
    EXPECT_EQ(Point{}, p);
  }

  cloud_wrapper.clear();
  EXPECT_TRUE(cloud_wrapper.empty());

  // Cannot reinitialize an already initialized message.
  EXPECT_THROW(PointCloud2Modifier<Point>(msg, "some_new_frame_id"), std::runtime_error);
  cloud_wrapper.reset_msg("some_new_frame_id");
  EXPECT_EQ(msg.header.frame_id, "some_new_frame_id");
  EXPECT_TRUE(cloud_wrapper.empty());
}

/// @test Check that using using iterator including std::back_inserter is possible with the wrapper.
TYPED_TEST(PointCloudMsgWrapperTest, Iterators) {
  using Point = typename TestFixture::PointT;
  sensor_msgs::msg::PointCloud2 msg;
  PointCloud2Modifier<Point> cloud_wrapper{msg, "some_frame_id"};
  std::array<Point, 2U> points{{create_point<Point>(), Point{}}};
  // Fill the message from an array.
  std::transform(
    points.cbegin(), points.cend(), std::back_inserter(cloud_wrapper),
    [](const Point & point) {return point;});
  ASSERT_EQ(points.size(), cloud_wrapper.size());
  for (auto i = 0U; i < points.size(); ++i) {
    EXPECT_EQ(points[i], cloud_wrapper[i]);
  }
  // Change existing values.
  for (auto & point : cloud_wrapper) {
    point = Point{};
  }
  const auto & const_cloud = msg;
  const PointCloud2View<Point> const_wrapper{const_cloud};
  for (const auto & point : const_wrapper) {
    EXPECT_EQ(Point{}, point);
  }

  // Check the reverse iteration.
  for (auto riter = const_wrapper.rbegin(); riter != const_wrapper.rend(); ++riter) {
    EXPECT_EQ(Point{}, *riter);
  }
}

/// @test Check initialization with a complicated type with an additional custom field.
TEST(PointCloudMsgWrapperTest, ProcessPointWithCustomField) {
  sensor_msgs::msg::PointCloud2 msg;
  // There is no matching field generator for non_standard_test_field present.
  EXPECT_THROW(PointCloud2Modifier<PointWithCustomField>(msg, "some_frame_id"), std::runtime_error);
  EXPECT_THROW(PointCloud2View<PointWithCustomField>{msg}, std::runtime_error);
  using Generators = std::tuple<
    point_cloud_msg_wrapper::field_x_generator,
    point_cloud_msg_wrapper::field_y_generator,
    field_non_standard_test_field_generator>;
  using CustomCloudModifier = PointCloud2Modifier<PointWithCustomField, Generators>;
  // Cannot initialize message without a new frame id provided.
  EXPECT_THROW(CustomCloudModifier{msg}, std::runtime_error);

  CustomCloudModifier cloud_wrapper{msg, "some_frame_id"};
  ASSERT_EQ(msg.fields.size(), 3U);
  // Note that the order of fields is defined by the order of generators in the tuple, NOT by the
  // order of members in the struct. However, the order of fields plays no role in point
  // representation within the message.
  EXPECT_EQ(msg.fields[0].name, "x");
  EXPECT_EQ(msg.fields[1].name, "y");
  EXPECT_EQ(msg.fields[2].name, "non_standard_test_field");
  EXPECT_NO_THROW(cloud_wrapper.push_back({42.0F, 42.0, 23}));

  const CustomCloudModifier initialized_wrapper{msg};
  ASSERT_FALSE(initialized_wrapper.empty());
  ASSERT_EQ(initialized_wrapper.size(), 1U);
  EXPECT_FLOAT_EQ(initialized_wrapper.at(0U).x, 42.0F);
  EXPECT_DOUBLE_EQ(initialized_wrapper.at(0U).non_standard_test_field, 42.0);
  EXPECT_EQ(initialized_wrapper.at(0U).y, 23);
  EXPECT_THROW(initialized_wrapper.at(1U), std::out_of_range);
  EXPECT_FLOAT_EQ(initialized_wrapper[0U].x, 42.0F);
  EXPECT_DOUBLE_EQ(initialized_wrapper[0U].non_standard_test_field, 42.0);
  EXPECT_EQ(initialized_wrapper[0U].y, 23);
  EXPECT_NO_THROW(cloud_wrapper.push_back(initialized_wrapper[0U]));
  ASSERT_EQ(initialized_wrapper.size(), 2U);
}

/// @test Check that a macro we use for readability is not leaking outside of the header file.
TEST(PointCloudMsgWrapperTest, CompilationMacroIsUnset) {
  #ifdef COMPILE_IF
  FAIL() << "Compilation macro should not be available outside of point_cloud_msg_wrapper.hpp file";
  #endif
}

/// @test Check that we can read partial data from a point cloud.
TEST(PointCloudMsgWrapperTest, PartialDataAccess) {
  sensor_msgs::msg::PointCloud2 msg;
  PointCloud2Modifier<PointXYZI> modifier{msg, "frame_id"};
  const auto point = create_point<PointXYZI>();
  modifier.push_back(point);
  modifier.push_back(point);
  EXPECT_FALSE(PointCloud2View<PointX>::can_be_created_from(msg));
  EXPECT_FALSE(PointCloud2Modifier<PointX>::can_be_created_from(msg));
  EXPECT_TRUE(PointCloud2PartialView<PointX>::can_be_created_from(msg));
  EXPECT_TRUE(PointCloud2PartialModifier<PointX>::can_be_created_from(msg));
  EXPECT_FALSE(PointCloud2PartialView<PointNotPresentInAllPointTypes>::can_be_created_from(msg));
  PointCloud2PartialView<PointX> partial_view{msg};
  EXPECT_FLOAT_EQ(partial_view[0].x, point.x);

  PointCloud2PartialModifier<PointX> partial_modifier{msg};
  partial_modifier[0].x = 0.0F;
  EXPECT_FLOAT_EQ(partial_view[0].x, 0.0F);
  PointCloud2View<PointXYZI> full_view{msg};
  EXPECT_FLOAT_EQ(full_view[0].x, 0.0F);
  EXPECT_FLOAT_EQ(full_view[0].y, point.y);
  EXPECT_FLOAT_EQ(full_view[0].z, point.z);
  EXPECT_EQ(full_view[0].id, point.id);

  for (auto & iter_point : partial_modifier) {
    iter_point.x = 1.0F;
  }
  auto modified_point = point;
  modified_point.x = 1.0F;
  PointX point_x{1.0F};
  EXPECT_EQ(full_view[0], modified_point);
  EXPECT_EQ(partial_view.front(), point_x);
  EXPECT_EQ(partial_modifier.front(), point_x);
  EXPECT_EQ(partial_view.back(), point_x);
  EXPECT_EQ(partial_modifier.back(), point_x);

  const auto point_xy = create_point<ClassPointXY>();
  PointCloud2PartialModifier<ClassPointXY>{msg}.front() = point_xy;
  EXPECT_EQ(full_view.front().x, point_xy.x());
  EXPECT_EQ(full_view.front().y, point_xy.y());
  EXPECT_EQ(full_view.front().z, modified_point.z);
  EXPECT_EQ(full_view.front().id, modified_point.id);

  sensor_msgs::msg::PointCloud2 msg_x;
  PointCloud2Modifier<PointX> modifier_x{msg_x, "frame_id"};
  std::transform(
    partial_view.begin(), partial_view.end(), std::back_inserter(modifier_x),
    [](const auto & point_x) {return point_x;});
  EXPECT_EQ(modifier_x.size(), 2UL);
  EXPECT_FLOAT_EQ(modifier_x.front().x, full_view.front().x);
  EXPECT_FLOAT_EQ(modifier_x.back().x, full_view.back().x);
}
