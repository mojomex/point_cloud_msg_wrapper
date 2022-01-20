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


#ifndef POINT_CLOUD_MSG_WRAPPER__POINT_CLOUD_MSG_WRAPPER_HPP_
#define POINT_CLOUD_MSG_WRAPPER__POINT_CLOUD_MSG_WRAPPER_HPP_

#include <point_cloud_msg_wrapper/default_field_generators.hpp>
#include <point_cloud_msg_wrapper/field_generators.hpp>
#include <point_cloud_msg_wrapper/type_traits.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <string>

#include <limits>
#include <memory>
#include <tuple>
#include <utility>

///
/// @brief      A help macro to simplify conditional compilation based on mutability.
///
/// @details    This macro just wraps the SFINAE paradigm and enables compilation only if the
///             CONDITION is true. This macro is replaced by RETURN_TYPE in this case. If
///             CONDITION is false, the code following this macro invocation will not be compiled.
///
/// @param      CONDITION    The condition which, if false, stops compilation
/// @param      RETURN_TYPE  The return type generated if compilation succeeds.
///
#define COMPILE_IF(CONDITION, RETURN_TYPE) \
  template<typename DummyCloudMsgT = CloudMsgT> \
  std::enable_if_t<!std::is_trivial<DummyCloudMsgT>::value && CONDITION, RETURN_TYPE> \

namespace point_cloud_msg_wrapper
{

namespace detail
{
using DefaultFieldGenerators = std::tuple<
  field_x_generator,
  field_y_generator,
  field_z_generator,
  field_id_generator,
  field_ring_generator,
  field_intensity_generator,
  field_timestamp_generator>;
}  // namespace detail

///
/// @brief      This class implements a point cloud message wrapper. Unless otherwise required, use
///             the typedefs of this class: PointCloud2View and PointCloud2Modifier. Only use this
///             class directly if those typedefs do not provide enough flexibility.
///
/// @details    This class is designed to simplify working with point cloud messages. The idea is
///             that it wraps a point cloud message reference allowing for simple access and
///             modification. Upon creation, the wrapper checks if the fields of the message
///             correspond to the fields generated from the point type provided by the user. If this
///             check has passed, then the fields are identical (including the offsets of the
///             members) and it is safe to reinterpret the point cloud message as an array of PointT
///             points. Note that due to these checks it is relatively expensive (the complexity of
///             the checks is around O(n^2) where n is the number of field generators) to create
///             this wrapper, thus it should be avoided in very tight scopes. This operation is not
///             slow, but is significantly slower than a single element access. It is better to
///             create the wrapper once per message in a function scope and batch all the read/write
///             operations afterwards.
///
///             For convenience, there are 4 typedefs:
///             - PointCloud2View - wraps a constant point cloud and allows read-only access.
///               Modification through this view is impossible.
///             - PointCloud2Modifier - wraps a mutable message and allows read-write access to the
///               underlying data. There is also a constructor through which the fields of the
///               underlying message can be generated from the given point structure.
///             - PointCloud2PartialView - wraps a constant point cloud and allows read-only
///               _partial_ access to the data.
///             - PointCloud2PartialModifier - wraps a mutable message and allows read-write
///               _partial_ access to the data, i.e., it allows to get a subset of the fields.
///
/// @warning    This class wraps a raw reference, so the user is responsible to use it in such a way
///             that the underlying point cloud message is not deleted before the wrapper.
///
/// @tparam     PointT           Type of point to use for message reading/writing.
/// @tparam     PointCloudMsgT   Type of point cloud message.
/// @tparam     FieldGenerators  A tuple of all field generators that allow generating Field structs
///                              from members of a PointT struct. See
///                              LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER for more details.
///                              The class provides a sane default value here, but a custom tuple
///                              can be generated by the user by using
///                              LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER if needed.
/// @tparam     kIsMutable       Define if the point cloud message is mutable. Used to conditionally
///                              compile certain functions.
/// @tparam     kPartialAccess   Define if the partial access to the data is allowed.
/// @tparam     AllocatorT       The message allocator type. This allocator type must be rebindable
///                              to the PointT allocator through the use of
///                              `std::allocator_traits<AllocatorT>::rebind_alloc<PointT>`.
///
template<
  typename PointT,
  template<typename AllocatorT> class PointCloudMsgT,
  typename FieldGenerators,
  const bool kIsMutable,
  const bool kPartialAccess,
  typename AllocatorT>
class PointCloudMsgWrapper
{
  /// Depending on the provided kIsMutable boolean use the const or a mutable point cloud msg type.
  using CloudMsgT =
    std::conditional_t<kIsMutable, PointCloudMsgT<AllocatorT>, const PointCloudMsgT<AllocatorT>>;

  static_assert(
    std::is_default_constructible<PointT>::value,
    "\n\nThe point type must satisfy is_default_constructible trait.\n\n");

  static_assert(
    detail::is_specialization<FieldGenerators, std::tuple>::value,
    "\n\nFieldGenerators must be an std::tuple.\n\n");

  static_assert(
    std::is_same<std::decay_t<CloudMsgT>, sensor_msgs::msg::PointCloud2_<AllocatorT>>::value,
    "\n\nThis class is designed to work with PointCloud2 messages only for now.\n\n");

  /// Type of the fields entry
  using FieldNameT = typename sensor_msgs::msg::PointField_<AllocatorT>::_name_type;
  /// Type of the data entry
  using DataVectorT = typename sensor_msgs::msg::PointCloud2_<AllocatorT>::_data_type;

  /// Get the type of the allocator to use with points.
  using PointAllocatorT = typename std::allocator_traits<AllocatorT>::template rebind_alloc<PointT>;

  /// Derive a point vector type (usually std::vector<PointT>). Needed in case a different (e.g.
  /// bounded vector) type used instead and use it for the Point type with the msg Allocator.
  using PointVectorType =
    typename detail::derived_point_vector<DataVectorT, PointT>::template type<PointAllocatorT>;

  /// Size of the point.
  using kSizeofPoint =
    std::integral_constant<std::uint32_t, static_cast<std::uint32_t>(sizeof(PointT))>;

public:
  ///
  /// @brief      A modifying iterator over the underlying point cloud message. Only relevant if the
  ///             point cloud type is not const.
  ///
  template<bool kIsConstIterator>
  class CustomIterator;

  using Iterator = CustomIterator<false>;
  using ConstIterator = CustomIterator<true>;

  using value_type = PointT;
  using iterator =
    std::conditional_t<kPartialAccess, Iterator, typename PointVectorType::iterator>;
  using const_iterator =
    std::conditional_t<kPartialAccess, ConstIterator, typename PointVectorType::const_iterator>;
  using reverse_iterator = std::conditional_t<
    kPartialAccess, std::reverse_iterator<Iterator>, typename PointVectorType::reverse_iterator>;
  using const_reverse_iterator = std::conditional_t<
    kPartialAccess, std::reverse_iterator<ConstIterator>,
    typename PointVectorType::const_reverse_iterator>;

  ///
  /// @brief      Constructor that wraps a point cloud message. Checks if the message fields
  ///             correspond to the ones generated from the PointT type and throws in case of a
  ///             mismatch.
  ///
  /// @param      cloud_ref  A reference to the wrapped cloud.
  ///
  explicit PointCloudMsgWrapper(CloudMsgT & cloud_ref)
  : m_cloud_ref{cloud_ref}
  {
    if (m_cloud_ref.fields.empty()) {
      throw std::runtime_error(
              "Trying to wrap a message with no fields. Reset this message with"
              "'create_wrapper_from_empty_msg' function instead!");
    }
    std::string error_msg{};
    if (!can_be_created_from(cloud_ref, &error_msg)) {
      throw std::runtime_error(error_msg);
    }
  }

  ///
  /// @brief      A constructor that initializes the message and sets a new frame_id. Note that this
  ///             only compiles for a mutable cloud. It also throws if the message is already
  ///             initialized.
  ///
  /// @param      cloud_ref  A reference to the wrapped cloud
  /// @param[in]  frame_id   A frame id to be set to the message.
  ///
  explicit PointCloudMsgWrapper(CloudMsgT & cloud_ref, const FieldNameT & frame_id)
  : m_cloud_ref{cloud_ref}
  {
    static_assert(
      kIsMutable, "\n\nWe must be able to modify this point cloud.\n\n");
    static_assert(
      detail::has_operator_equals<PointT>::value,
      "\n\nTo guarantee that all struct members are present in the fields of the message, "
      "the point struct needs an equality operator defined.\n\n");
    static_assert(
      sizeof(PointT) < static_cast<std::size_t>(UINT32_MAX),
      "\n\nOnly points with sizeof that fits in uint32_t are supported\n\n");
    static_assert(
      !kPartialAccess,
      "\n\nThis constructor is not available when allowing partial access to the data\n\n");

    if (!m_cloud_ref.fields.empty()) {
      throw std::runtime_error(
              "Trying to reset a non-empty point cloud message."
              " Use reset_msg with the correct wrapper instead.");
    }
    const auto generated_fields = generate_fields_from_point<PointT, FieldGenerators>();
    if (!check_that_generated_fields_cover_all_point_members(generated_fields)) {
      throw std::runtime_error(
              "Generated fields don't match the members of the point struct. "
              "Make sure you have correct generators tuple passed into this class.");
    }
    reset_msg(frame_id, generated_fields);
  }


  ///
  /// @brief      Determines ability to be created from a given cloud.
  ///
  /// @param[in]  cloud_msg  The cloud message
  /// @param      error_msg  Optional error message
  ///
  /// @return     True if able to be created for the provided cloud, False otherwise.
  ///
  static bool can_be_created_from(const CloudMsgT & cloud_msg, std::string * error_msg = nullptr)
  {
    const auto find_missing_field = [](
      const auto & query_fields,
      const auto & source_fields) -> const FieldNameT * {
        for (const auto & query_field : query_fields) {
          // Note that we use find on a vector here. This is intended. The number of fields is
          // usually very limited, so the O(n^2) complexity is ok here. This operation also only
          // takes place roughly once per message. The subsequent read/write operations are then
          // appropriately fast.
          const auto corresponding_field_iter = std::find_if(
            source_fields.begin(), source_fields.end(), [&query_field](const auto & field) {
              const bool equal{
                (field.name == query_field.name) && (field.count == query_field.count) &&
                (field.datatype == query_field.datatype) && (field.offset == query_field.offset)};
              return equal;
            });
          const auto found_corresponding_field = (corresponding_field_iter != source_fields.end());
          if (!found_corresponding_field) {return &query_field.name;}
        }
        return nullptr;
      };

    const auto struct_fields = generate_fields_from_point<PointT, FieldGenerators>();
    const auto missing_field_in_cloud = find_missing_field(struct_fields, cloud_msg.fields);
    if (missing_field_in_cloud) {
      if (error_msg) {
        *error_msg = "Point struct has a field that the cloud does not! Field: " +
          *missing_field_in_cloud;
      }
      return false;
    }
    if (kPartialAccess) {
      // We skip some checks here and expect the user to know what they are doing. If the user did
      // not specify the correct generators for the point structure, there is no check for this. If
      // there is no generator for a certain field name, this field will be silently ignored when
      // reading the data, thus the data in this field while reading/writing to the point cloud
      // message will be wrong.
      return true;
    }
    const auto missing_field_in_struct = find_missing_field(cloud_msg.fields, struct_fields);
    if (missing_field_in_struct) {
      if (error_msg) {
        *error_msg = "Cloud has a field that the point struct does not! Field: " +
          *missing_field_in_struct;
      }
      return false;
    }
    if (cloud_msg.point_step != kSizeofPoint::value) {
      if (error_msg) {
        *error_msg =
          "Point cloud was created with a point of different sizeof. "
          "Are the point members in the same order? Cloud point step = " + std::to_string(
          cloud_msg.point_step) + " while sizeof(PointT) = " + std::to_string(sizeof(PointT));
      }
      return false;
    }
    if (sizeof(PointT) * cloud_msg.width != cloud_msg.data.size()) {
      if (error_msg) {
        *error_msg =
          "Point cloud data size " + std::to_string(cloud_msg.data.size()) +
          " does not match the data size derived from the point size of count, which is " +
          std::to_string(sizeof(PointT) * cloud_msg.width);
      }
      return false;
    }
    return true;
  }

  /// @brief      Push a new point into the message.
  COMPILE_IF(kIsMutable, void) push_back(const PointT & point)
  {
    PointT point_copy{point};
    extend_data_by(m_cloud_ref.point_step);
    m_cloud_ref.row_step += m_cloud_ref.point_step;
    m_cloud_ref.width++;
    this->operator[](m_cloud_ref.width - 1U) = point_copy;
  }

  /// @brief      Push a new point into the message.
  COMPILE_IF(kIsMutable, void) push_back(PointT && point)
  {
    extend_data_by(m_cloud_ref.point_step);
    m_cloud_ref.row_step += m_cloud_ref.point_step;
    m_cloud_ref.width++;
    this->operator[](m_cloud_ref.width - 1U) = std::move(point);
  }

  /// Get the number of points in the message.
  std::size_t size() const noexcept {return m_cloud_ref.width;}

  /// Check if the point cloud message stores no points.
  bool empty() const noexcept {return m_cloud_ref.width == 0U;}

  /// Get the first point.
  COMPILE_IF(kIsMutable, PointT &) front() noexcept {return *begin();}
  /// Get the first point.
  const PointT & front() const noexcept {return *begin();}
  /// Get the last point.
  COMPILE_IF(kIsMutable, PointT &) back() noexcept {return *rbegin();}
  /// Get the last point.
  const PointT & back() const noexcept {return *rbegin();}

  /// Get a point reference at the specified index.
  /// @throws std::runtime_error if the index is out of bounds.
  const PointT & at(const std::size_t index) const
  {
    if (index >= size()) {
      throw std::out_of_range(
              "Index is out of bounds, " +
              std::to_string(index) + " >= " + std::to_string(size()));
    }
    return reinterpret_cast<const PointT &>(
      *(m_cloud_ref.data.data() + index * m_cloud_ref.point_step));
  }
  /// Get a point reference at the specified index. Only compiled if message is not const.
  /// @throws std::runtime_error if the index is out of bounds.
  COMPILE_IF(kIsMutable, PointT &) at(const std::size_t index)
  {
    if (index >= size()) {
      throw std::out_of_range(
              "Index is out of bounds, " +
              std::to_string(index) + " >= " + std::to_string(size()));
    }
    return reinterpret_cast<PointT &>(*(m_cloud_ref.data.data() + index * m_cloud_ref.point_step));
  }

  /// Get a point reference at the specified index.
  const PointT & operator[](const std::size_t index) const noexcept
  {
    return *reinterpret_cast<const PointT * const>(
      m_cloud_ref.data.data() + index * m_cloud_ref.point_step);
  }
  /// Get a point reference as a specified index.  Only compiled if message type is not const.
  COMPILE_IF(kIsMutable, PointT &) operator[](const std::size_t index) noexcept
  {
    return *reinterpret_cast<PointT *>(m_cloud_ref.data.data() + index * m_cloud_ref.point_step);
  }

  /// @brief      Reset the message fields to match the members of the PointT struct. The point
  ///             cloud message is ready for modification after this operation.
  COMPILE_IF(kIsMutable, void) reset_msg(const FieldNameT & frame_id)
  {
    reset_msg(frame_id, generate_fields_from_point<PointT, FieldGenerators>());
  }

  /// @brief      Clear the message.
  COMPILE_IF(kIsMutable, void) clear() {
    reset_msg(m_cloud_ref.header.frame_id);
  }

  /// @brief      Allocate memory to hold a specified number of points.
  COMPILE_IF(kIsMutable, void) reserve(const size_t expected_number_of_points) {
    m_cloud_ref.data.reserve(m_cloud_ref.point_step * expected_number_of_points);
  }

  /// @brief      Resize the container to hold a given number of points.
  COMPILE_IF(kIsMutable, void) resize(const std::uint32_t new_number_of_points) {
    m_cloud_ref.width = new_number_of_points;
    m_cloud_ref.row_step = m_cloud_ref.width * m_cloud_ref.point_step;
    m_cloud_ref.data.resize(m_cloud_ref.row_step);
  }

  /// An iterator to the beginning of data.  Only compiled if point cloud message type is not const.
  COMPILE_IF(kIsMutable && !kPartialAccess, iterator) begin() noexcept {
    return iterator{reinterpret_cast<PointT *>(&m_cloud_ref.data[0U])};
  }

  /// An iterator to the beginning of data.  Only compiled if point cloud message type is not const.
  COMPILE_IF(kIsMutable && kPartialAccess, iterator) begin() noexcept {
    return iterator{this, 0U};
  }

  /// An iterator to the end of data. Only compiled if point cloud message type is not const.
  COMPILE_IF(kIsMutable && !kPartialAccess, iterator) end() noexcept {
    return iterator{reinterpret_cast<PointT *>(&m_cloud_ref.data[m_cloud_ref.data.size()])};
  }
  /// An iterator to the end of data. Only compiled if point cloud message type is not const.
  COMPILE_IF(kIsMutable && kPartialAccess, iterator) end() noexcept {
    return iterator{this, size()};
  }

  /// A reverse iterator staring position.  Only compiled if point cloud message type is not const.
  COMPILE_IF(kIsMutable, reverse_iterator) rbegin() noexcept {
    return std::make_reverse_iterator(end());
  }
  /// A reverse iterator end position.  Only compiled if point cloud message type is not const.
  COMPILE_IF(kIsMutable, reverse_iterator) rend() noexcept {
    return std::make_reverse_iterator(begin());
  }
  /// A constant iterator to the beginning of data.
  const_iterator begin() const noexcept {return cbegin();}
  /// A constant iterator to the end of data.
  const_iterator end() const noexcept {return cend();}
  /// A constant reverse iterator to the beginning of the reversed data.
  const_reverse_iterator rbegin() const noexcept {return crbegin();}
  /// A constant reverse iterator to the end of reversed data.
  const_reverse_iterator rend() const noexcept {return crend();}
  /// A constant iterator to the beginning of data.
  COMPILE_IF(!kPartialAccess, const_iterator) cbegin() const noexcept
  {
    return const_iterator{reinterpret_cast<const PointT * const>(&m_cloud_ref.data[0U])};
  }
  COMPILE_IF(kPartialAccess, const_iterator) cbegin() const noexcept
  {
    return const_iterator{this, 0U};
  }
  ///
  /// @brief      A constant iterator to the end of data.
  ///
  /// @note       Only compiled if partial access is **not** allowed.
  ///
  /// @details    This reinterprets the underlying data as a vector of points and provides a
  ///             standard underlying vector iterator.
  ///
  COMPILE_IF(!kPartialAccess, const_iterator) cend() const noexcept
  {
    return const_iterator{
      reinterpret_cast<const PointT * const>(&m_cloud_ref.data[m_cloud_ref.data.size()])};
  }
  ///
  /// @brief      A constant iterator to the end of data.
  ///
  /// @note       Only compiled if partial access is allowed.
  ///
  /// @details    This returns a custom iterator that allows for partial access to the data.
  ///
  COMPILE_IF(kPartialAccess, const_iterator) cend() const noexcept
  {
    return const_iterator{this, size()};
  }
  /// A constant reverse iterator to the reverse beginning of data.
  const_reverse_iterator crbegin() const noexcept {return std::make_reverse_iterator(cend());}
  /// A constant reverse iterator to the reverse end of data.
  const_reverse_iterator crend() const noexcept {return std::make_reverse_iterator(cbegin());}

private:
  /// Allocate additional memory in the end of data field of the message.
  COMPILE_IF(kIsMutable, void) extend_data_by(const std::size_t bytes_to_allocate)
  {
    const auto new_size = m_cloud_ref.data.size() + bytes_to_allocate;
    m_cloud_ref.data.resize(new_size);
  }

  /// @brief      Reset the message fields to match the members of the PointT struct. The point
  ///             cloud message is ready for modification after this operation.
  COMPILE_IF(kIsMutable, void) reset_msg(
    const FieldNameT & frame_id,
    const sensor_msgs::msg::PointCloud2::_fields_type & generated_fields)
  {
    // TODO(igor): All these settings can also be specified through some input struct if needed in
    // the future. This is omitted for now.
    m_cloud_ref.fields = generated_fields;
    m_cloud_ref.height = 1;
    m_cloud_ref.is_bigendian = false;
    m_cloud_ref.is_dense = false;
    m_cloud_ref.header.frame_id = frame_id;
    m_cloud_ref.point_step = kSizeofPoint::value;
    m_cloud_ref.data.clear();
    m_cloud_ref.width = 0U;
    m_cloud_ref.row_step = 0U;
  }

  ///
  /// @brief      Check that the generated fields actually cover all the point members.
  ///
  /// @details    This function uses a clever trick to check if all the fields of the point are
  ///             covered by the generated fields. The idea is the following:
  ///             1. Allocate storage as big as a point.
  ///             2. Fill the storage with zeros.
  ///             3. Iterate through all fields and set the appropriate memory to ones.
  ///             4. Flip all bits - now, if all fields are present, only the padding is non-zero.
  ///             5. Reinterpret this memory as a point.
  ///             6. Compare this point to a value initialized one. If these points are equal - we
  ///                have all fields. Otherwise, we are missing a field.
  ///
  /// @param[in]  generated_fields  The generated fields
  ///
  /// @return     True if all point members are covered and false otherwise.
  ///
  bool check_that_generated_fields_cover_all_point_members(
    const sensor_msgs::msg::PointCloud2::_fields_type & generated_fields)
  {
    std::uint32_t sum_of_all_fields_sizes{};
    for (const auto & field : generated_fields) {
      sum_of_all_fields_sizes += field.count * sizeof_field(field.datatype);
    }
    if (sum_of_all_fields_sizes == kSizeofPoint::value) {
      // The generated fields cover all members of the struct without gaps.
      return true;
    }
    // The idea is the following:
    // 1. Allocate storage as big as a point.
    // 2. Fill the storage with zeros.
    // 3. Iterate through all fields and set the appropriate memory to ones.
    // 4. Flip all bits - now, if all fields are present, only the padding is non-zero.
    // 5. Reinterpret this memory as a point.
    // 6. Compare this point to a value initialized one.
    // If these points are equal - we have all fields. Otherwise, we are missing a field.
    alignas(PointT) std::uint8_t buffer[sizeof(PointT)];
    std::fill(buffer, buffer + sizeof(PointT), 0U);
    for (const auto & field : generated_fields) {
      const auto field_first_byte = field.offset;
      const auto field_last_byte =
        field.offset + field.count * static_cast<std::size_t>(sizeof_field(field.datatype));
      std::fill(
        &buffer[field_first_byte],
        &buffer[field_last_byte],
        std::numeric_limits<std::uint8_t>::max());
    }
    for (auto i = 0U; i < sizeof(PointT); ++i) {
      buffer[i] = buffer[i] > 0 ? 0 : std::numeric_limits<std::uint8_t>::max();  // Flip all bits.
    }
    const PointT * const point_from_flipped_memory{reinterpret_cast<PointT *>(buffer)};
    if (PointT{} == *point_from_flipped_memory) {
      return true;
    }
    return false;
  }

  /// A reference to the cloud message.
  CloudMsgT & m_cloud_ref;
};

///
/// @brief      This class describes a point cloud message wrapper iterator.
///
/// @details    This iterator is used only if the partial access to the data is allowed. It is
///             designed to still be compliant to the standard algorithms. It steps over the data in
///             steps of message width. This iterator is used both for a normal iterator as well as
///             for const iterator through the use of a boolean template parameter.
///
/// @tparam     PointT            Point type
/// @tparam     PointCloudMsgT    Point cloud message type
/// @tparam     FieldGenerators   Field generators
/// @tparam     kIsMutable        Is the cloud mutable
/// @tparam     kPartialAccess    Is partial access allowed
/// @tparam     AllocatorT        An allocator
/// @tparam     kIsConstIterator  Is this a const iterator
///
template<
  typename PointT,
  template<typename AllocatorT> class PointCloudMsgT,
  typename FieldGenerators,
  const bool kIsMutable,
  const bool kPartialAccess,
  typename AllocatorT>
template<bool kIsConstIterator>
class PointCloudMsgWrapper<
    PointT, PointCloudMsgT, FieldGenerators, kIsMutable, kPartialAccess, AllocatorT>::CustomIterator
{
  using WrapperPtr =
    std::conditional_t<kIsConstIterator, const PointCloudMsgWrapper *, PointCloudMsgWrapper *>;

public:
  using iterator_category = std::random_access_iterator_tag;
  using value_type = PointT;
  using difference_type = std::ptrdiff_t;
  using reference = std::conditional_t<kIsConstIterator, const PointT &, PointT &>;
  using pointer = std::conditional_t<kIsConstIterator, const PointT * const, PointT *>;

  ///
  /// @brief      Constructs a new instance of an iterator.
  ///
  explicit CustomIterator(WrapperPtr wrapper, std::size_t index)
    : m_wrapper {wrapper}, m_index{index} {}

  ///
  /// @brief      Get the underlying value.
  ///
  COMPILE_IF(!kIsConstIterator, reference) operator*() noexcept {
    return m_wrapper->operator[](m_index);
  }
  ///
  /// @brief      Get the underlying value.
  ///
  COMPILE_IF(kIsConstIterator, reference) operator*() const noexcept {
    return m_wrapper->operator[](m_index);
  }

  ///
  /// @brief      Increment operator.
  ///
  /// @return     The result of the increment
  ///
  CustomIterator & operator++() noexcept
  {
    ++m_index;
    return *this;
  }

  ///
  /// @brief      Increment operator.
  ///
  /// @return     The result of the increment
  ///
  CustomIterator operator++(int) noexcept
  {
    CustomIterator past = *this;
    ++(*this);
    return past;
  }

  ///
  /// @brief      Decrement operator.
  ///
  /// @return     The result of the decrement
  ///
  CustomIterator & operator--() noexcept
  {
    --m_index;
    return *this;
  }

  ///
  /// @brief      Decrement operator.
  ///
  /// @return     The result of the decrement
  ///
  CustomIterator operator--(int) noexcept
  {
    CustomIterator past = *this;
    --(*this);
    return past;
  }

  ///
  /// @brief      Equality operator.
  ///
  /// @param[in]  other  The other
  ///
  /// @return     The result of the equality
  ///
  bool operator==(const CustomIterator & other) const noexcept
  {
    return (m_wrapper == other.m_wrapper) && (m_index == other.m_index);
  }

  ///
  /// @brief      Inequality operator.
  ///
  /// @param[in]  other  The other
  ///
  /// @return     The result of the inequality
  ///
  bool operator!=(const CustomIterator & other) const noexcept
  {
    return !(*this == other);
  }

private:
  WrapperPtr m_wrapper{};
  std::size_t m_index{};
};


/// A typedef for the PointCloudMsgWrapper to represent a view that wraps a const cloud message.
template<
  typename PointT,
  typename FieldGeneratorsT = detail::DefaultFieldGenerators,
  typename AllocatorT = std::allocator<void>>
using PointCloud2View = PointCloudMsgWrapper<
  PointT, sensor_msgs::msg::PointCloud2_, FieldGeneratorsT, false, false, AllocatorT>;

/// A typedef for the PointCloudMsgWrapper to represent a view that wraps a mutable cloud message.
template<
  typename PointT,
  typename FieldGeneratorsT = detail::DefaultFieldGenerators,
  typename AllocatorT = std::allocator<void>>
using PointCloud2Modifier = PointCloudMsgWrapper<
  PointT, sensor_msgs::msg::PointCloud2_, FieldGeneratorsT, true, false, AllocatorT>;

///
/// @brief      A typedef for the PointCloudMsgWrapper to represent a view that wraps a const cloud
///             message that allows partial data access.
///
/// @details    Partial data access allows getting only part of the data that is stored in the point
///             cloud message. The limitation to such access is that the data type, names and
///             offsets of the subset of the fields should still match the ones stored in the point
///             cloud message. So it is possible to read PointXY from a point cloud that contains
///             data generated from PointXYZ structs, but it is not possible to get PointXZ (Y
///             skipped) data.
///
template<
  typename PointT,
  typename FieldGeneratorsT = detail::DefaultFieldGenerators,
  typename AllocatorT = std::allocator<void>>
using PointCloud2PartialView = PointCloudMsgWrapper<
  PointT, sensor_msgs::msg::PointCloud2_, FieldGeneratorsT, false, true, AllocatorT>;

///
/// @brief      A typedef for the PointCloudMsgWrapper to represent a view that wraps a mutable
///             cloud message that allows partial data access.
///
/// @details    Partial data access allows getting only part of the data that is stored in the point
///             cloud message. The limitation to such access is that the data type, names and
///             offsets of the subset of the fields should still match the ones stored in the point
///             cloud message. So it is possible to read PointXY from a point cloud that contains
///             data generated from PointXYZ structs, but it is not possible to get PointXZ (Y
///             skipped) data.
///
/// @note       This typedef does _not_ allow initializing a new point cloud message.
///
template<
  typename PointT,
  typename FieldGeneratorsT = detail::DefaultFieldGenerators,
  typename AllocatorT = std::allocator<void>>
using PointCloud2PartialModifier = PointCloudMsgWrapper<
  PointT, sensor_msgs::msg::PointCloud2_, FieldGeneratorsT, true, true, AllocatorT>;

}  // namespace point_cloud_msg_wrapper

#undef COMPILE_IF

#endif  // POINT_CLOUD_MSG_WRAPPER__POINT_CLOUD_MSG_WRAPPER_HPP_
