// Copyright 2026 Juan Carlos Manzanares Serrano
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

#include "plansys2_state_updater/message_reader.hpp"

#include <cstdint>
#include <limits>
#include <memory>
#include <new>
#include <stdexcept>
#include <string>

#include "rclcpp/serialization.hpp"
#include "rclcpp/typesupport_helpers.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"

namespace plansys2_state_updater
{

namespace
{

namespace intro = rosidl_typesupport_introspection_cpp;

Value read_members(const void * message, const intro::MessageMembers * members);

Value
unsigned_value(uint64_t value)
{
  // Must not wrap past int64
  if (value > static_cast<uint64_t>(std::numeric_limits<int64_t>::max())) {
    return Value(static_cast<double>(value));
  }
  return Value(static_cast<int64_t>(value));
}

std::string
utf8(const std::u16string & text)
{
  std::string out;
  for (char16_t c : text) {
    if (c < 0x80) {
      out += static_cast<char>(c);
    } else if (c < 0x800) {
      out += static_cast<char>(0xC0 | (c >> 6));
      out += static_cast<char>(0x80 | (c & 0x3F));
    } else {
      out += static_cast<char>(0xE0 | (c >> 12));
      out += static_cast<char>(0x80 | ((c >> 6) & 0x3F));
      out += static_cast<char>(0x80 | (c & 0x3F));
    }
  }
  return out;
}

Value
read_element(const void * element, const intro::MessageMember & member)
{
  switch (member.type_id_) {
    case intro::ROS_TYPE_FLOAT: return Value(*static_cast<const float *>(element));
    case intro::ROS_TYPE_DOUBLE: return Value(*static_cast<const double *>(element));
    case intro::ROS_TYPE_LONG_DOUBLE:
      return Value(static_cast<double>(*static_cast<const long double *>(element)));
    case intro::ROS_TYPE_CHAR: return Value(*static_cast<const unsigned char *>(element));
    case intro::ROS_TYPE_WCHAR: return Value(*static_cast<const char16_t *>(element));
    case intro::ROS_TYPE_BOOLEAN: return Value(*static_cast<const bool *>(element));
    case intro::ROS_TYPE_OCTET: return Value(*static_cast<const unsigned char *>(element));
    case intro::ROS_TYPE_UINT8: return Value(*static_cast<const uint8_t *>(element));
    case intro::ROS_TYPE_INT8: return Value(*static_cast<const int8_t *>(element));
    case intro::ROS_TYPE_UINT16: return Value(*static_cast<const uint16_t *>(element));
    case intro::ROS_TYPE_INT16: return Value(*static_cast<const int16_t *>(element));
    case intro::ROS_TYPE_UINT32: return Value(*static_cast<const uint32_t *>(element));
    case intro::ROS_TYPE_INT32: return Value(*static_cast<const int32_t *>(element));
    case intro::ROS_TYPE_UINT64: return unsigned_value(*static_cast<const uint64_t *>(element));
    case intro::ROS_TYPE_INT64: return Value(*static_cast<const int64_t *>(element));
    case intro::ROS_TYPE_STRING: return Value(*static_cast<const std::string *>(element));
    case intro::ROS_TYPE_WSTRING:
      return Value(utf8(*static_cast<const std::u16string *>(element)));
    case intro::ROS_TYPE_MESSAGE:
      return read_members(
        element, static_cast<const intro::MessageMembers *>(member.members_->data));
    default:
      return Value();
  }
}

Value
read_field(const void * field, const intro::MessageMember & member)
{
  if (!member.is_array_) {
    return read_element(field, member);
  }
  Value::List list;
  const size_t size = member.size_function ? member.size_function(field) : member.array_size_;
  const bool primitive = member.type_id_ != intro::ROS_TYPE_STRING &&
    member.type_id_ != intro::ROS_TYPE_WSTRING && member.type_id_ != intro::ROS_TYPE_MESSAGE;
  for (size_t i = 0; i < size; i++) {
    if (primitive && member.fetch_function) {
      // Elements of a bool sequence have no address: fetch them by value
      alignas(std::max_align_t) unsigned char buffer[sizeof(long double)] = {};
      member.fetch_function(field, i, buffer);
      list.push_back(read_element(buffer, member));
    } else {
      list.push_back(read_element(member.get_const_function(field, i), member));
    }
  }
  return Value(std::move(list));
}

Value
read_members(const void * message, const intro::MessageMembers * members)
{
  Value::Map map;
  for (uint32_t i = 0; i < members->member_count_; i++) {
    const auto & member = members->members_[i];
    const void * field = static_cast<const uint8_t *>(message) + member.offset_;
    map[member.name_] = read_field(field, member);
  }
  return Value(std::move(map));
}

}  // namespace

MessageReader::MessageReader(const std::string & type)
: type_(type)
{
  try {
    library_ = rclcpp::get_typesupport_library(type, "rosidl_typesupport_cpp");
    type_support_ = rclcpp::get_message_typesupport_handle(
      type, "rosidl_typesupport_cpp", *library_);
    introspection_library_ = rclcpp::get_typesupport_library(
      type, "rosidl_typesupport_introspection_cpp");
    const auto * introspection = rclcpp::get_message_typesupport_handle(
      type, "rosidl_typesupport_introspection_cpp", *introspection_library_);
    members_ = static_cast<const intro::MessageMembers *>(introspection->data);
  } catch (const std::exception & e) {
    throw std::runtime_error("unknown type '" + type + "': " + e.what());
  }
}

Value
MessageReader::read(const rclcpp::SerializedMessage & serialized) const
{
  const std::align_val_t alignment{alignof(std::max_align_t)};
  void * message = ::operator new(members_->size_of_, alignment);
  members_->init_function(message, rosidl_runtime_cpp::MessageInitialization::ALL);
  try {
    rclcpp::SerializationBase(type_support_).deserialize_message(&serialized, message);
    Value value = read_members(message, members_);
    members_->fini_function(message);
    ::operator delete(message, alignment);
    return value;
  } catch (...) {
    members_->fini_function(message);
    ::operator delete(message, alignment);
    throw;
  }
}

}  // namespace plansys2_state_updater
