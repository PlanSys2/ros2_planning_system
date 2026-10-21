// Copyright 2026 Intelligent Robotics Lab
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

#ifndef PLANSYS2_CORE__PACKAGEPATHS_HPP_
#define PLANSYS2_CORE__PACKAGEPATHS_HPP_

#include <filesystem>
#include <string>

// Humble has only the string forms, which newer distros deprecate
#if __has_include("ament_index_cpp/get_package_share_path.hpp")
#define PLANSYS2_AMENT_INDEX_PATHS 1
#include "ament_index_cpp/get_package_prefix.hpp"
#include "ament_index_cpp/get_package_share_path.hpp"
#else
#include "ament_index_cpp/get_package_prefix.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#endif

namespace plansys2
{

// The share folder of a package
inline std::filesystem::path share_path(const std::string & package)
{
#ifdef PLANSYS2_AMENT_INDEX_PATHS
  return ament_index_cpp::get_package_share_path(package);
#else
  return ament_index_cpp::get_package_share_directory(package);
#endif
}

// The install prefix of a package
inline std::filesystem::path package_prefix(const std::string & package)
{
#ifdef PLANSYS2_AMENT_INDEX_PATHS
  std::filesystem::path prefix;
  ament_index_cpp::get_package_prefix(package, prefix);
  return prefix;
#else
  return ament_index_cpp::get_package_prefix(package);
#endif
}

}  // namespace plansys2

#endif  // PLANSYS2_CORE__PACKAGEPATHS_HPP_
