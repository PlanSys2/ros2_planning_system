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

#ifndef PLANSYS2_STATE_UPDATER__MESSAGE_READER_HPP_
#define PLANSYS2_STATE_UPDATER__MESSAGE_READER_HPP_

#include <memory>
#include <string>

#include "rclcpp/serialized_message.hpp"
#include "rcpputils/shared_library.hpp"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"

#include "plansys2_state_updater/value.hpp"

namespace plansys2_state_updater
{

/// Reads a serialized message of a type known only by name into a Value, by introspection.
/// Arrays become lists, nested messages maps, and bytes numbers (not characters).
class MessageReader
{
public:
  /// @throws std::runtime_error if the type is not known
  explicit MessageReader(const std::string & type);

  const std::string & type() const {return type_;}

  Value read(const rclcpp::SerializedMessage & serialized) const;

private:
  std::string type_;
  // The libraries must stay loaded while their type supports are used
  std::shared_ptr<rcpputils::SharedLibrary> library_;
  std::shared_ptr<rcpputils::SharedLibrary> introspection_library_;
  const rosidl_message_type_support_t * type_support_ {nullptr};
  const rosidl_typesupport_introspection_cpp::MessageMembers * members_ {nullptr};
};

}  // namespace plansys2_state_updater

#endif  // PLANSYS2_STATE_UPDATER__MESSAGE_READER_HPP_
