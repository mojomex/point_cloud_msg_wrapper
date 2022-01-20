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


#ifndef POINT_CLOUD_MSG_WRAPPER__TEST_POINT_TYPES_HPP_
#define POINT_CLOUD_MSG_WRAPPER__TEST_POINT_TYPES_HPP_

#include <geometry_msgs/msg/point32.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <type_traits>

namespace point_cloud_msg_wrapper
{
namespace test
{


namespace detail
{
template<typename T,
  std::enable_if_t<std::is_floating_point<T>::value> * = nullptr>
constexpr bool nearly_equal(
  const T & a,
  const T & b,
  const T & epsilon = std::numeric_limits<T>::epsilon()) noexcept
{
  return std::fabs(a - b) <=
         (epsilon * std::max(std::fabs(a), std::fabs(b)));
}
}  // namespace detail

using GeometryPointXYZ = geometry_msgs::msg::Point32;

struct PointWithCustomField
{
  float x;
  double non_standard_test_field;
  std::int32_t y;
  friend bool operator==(const PointWithCustomField & p1, const PointWithCustomField & p2) noexcept
  {
    return
      detail::nearly_equal(p1.x, p2.x) &&
      detail::nearly_equal(p1.non_standard_test_field, p2.non_standard_test_field) &&
      (p1.y == p2.y);
  }
};

struct CustomAlignedPoint
{
  float x;
  float y;
  float z;
  alignas(double) std::uint8_t intensity;
  double timestamp;
  friend bool operator==(const CustomAlignedPoint & p1, const CustomAlignedPoint & p2) noexcept
  {
    return
      detail::nearly_equal(p1.x, p2.x) &&
      detail::nearly_equal(p1.y, p2.y) &&
      detail::nearly_equal(p1.z, p2.z) &&
      detail::nearly_equal(p1.timestamp, p2.timestamp) &&
      (p1.intensity == p2.intensity);
  }
};

struct PointNotPresentInAllPointTypes {std::int8_t x;};

struct PointX
{
  float x;
  friend bool operator==(const PointX & p1, const PointX & p2) noexcept
  {
    return detail::nearly_equal(p1.x, p2.x);
  }
};

class ClassPointXY
{
public:
  ClassPointXY() = default;
  explicit ClassPointXY(float x, float y)
  : x_{x}, y_{y} {}

  float & x() {return x_;}
  const float & x() const {return x_;}
  float & y() {return y_;}
  const float & y() const {return y_;}

  friend bool operator==(const ClassPointXY & p1, const ClassPointXY & p2) noexcept
  {
    return detail::nearly_equal(p1.x_, p2.x_) &&
           detail::nearly_equal(p1.y_, p2.y_);
  }

private:
  float x_;
  float y_;
};

struct PointXYZI
{
  float x;
  float y;
  float z;
  std::int64_t id;
  friend bool operator==(const PointXYZI & p1, const PointXYZI & p2) noexcept
  {
    return detail::nearly_equal(p1.x, p2.x) &&
           detail::nearly_equal(p1.y, p2.y) &&
           detail::nearly_equal(p1.z, p2.z) &&
           (p1.id == p2.id);
  }
};

struct PointXYWithVirtualDestructor
{
  PointXYWithVirtualDestructor() = default;
  PointXYWithVirtualDestructor(float x_, float y_)
  : x{x_}, y{y_} {}

  virtual ~PointXYWithVirtualDestructor() {}

  friend bool operator==(
    const PointXYWithVirtualDestructor & p1,
    const PointXYWithVirtualDestructor & p2) noexcept
  {
    return detail::nearly_equal(p1.x, p2.x) &&
           detail::nearly_equal(p1.y, p2.y);
  }

  float x{};
  float y{};
};


template<typename PointT>
inline PointT create_point() {return PointT{};}

template<>
inline PointX create_point() {return PointX{42.0F};}
template<>
inline PointXYZI create_point() {return PointXYZI{42.0F, 23.0F, 13.0F, 42LL};}
template<>
inline ClassPointXY create_point() {return ClassPointXY{42.0F, 23.0F};}
template<>
inline PointXYWithVirtualDestructor create_point()
{
  return PointXYWithVirtualDestructor{42.0F, 23.0F};
}
template<>
inline GeometryPointXYZ create_point()
{
  GeometryPointXYZ point;
  point.set__x(42.0F).set__y(23.0F).set__x(13.0F);
  return point;
}
template<>
inline CustomAlignedPoint create_point()
{
  return CustomAlignedPoint{42.0F, 23.0F, 4242.0F, 23, 2323.0};
}


}  // namespace test
}  // namespace point_cloud_msg_wrapper

#endif  // POINT_CLOUD_MSG_WRAPPER__TEST_POINT_TYPES_HPP_
