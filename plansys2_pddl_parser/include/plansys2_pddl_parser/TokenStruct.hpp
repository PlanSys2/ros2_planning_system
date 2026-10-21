// Copyright 2024 Intelligent Robotics Lab
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
#pragma once
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "plansys2_pddl_parser/Basic.hpp"
/*
	T is either a pointer or a string
	A token structure stores pointers but does not delete them!
*/

namespace parser
{
namespace pddl
{

typedef std::map<std::string, unsigned> TokenMap;

template<typename T>
inline std::string getName(const T & t)
{
  return t->name;
}

template<>
inline std::string getName<std::string>(const std::string & s)
{
  return s;
}

template<typename T>
class TokenStruct
{
public:
  std::vector<T> tokens;
  TokenMap tokenMap;

  // represents the types of a typed list
  StringVec types;

  TokenStruct() {}

  TokenStruct(const TokenStruct & ts)
  : tokens(ts.tokens), tokenMap(ts.tokenMap), types(ts.types)
  {
  }

  TokenStruct & operator=(const TokenStruct & ts)
  {
    if (this != &ts) {
      tokens = ts.tokens;
      tokenMap = ts.tokenMap;
      types = ts.types;
    }
    return *this;
  }

  void append(const TokenStruct & ts)
  {
    for (unsigned i = 0; i < ts.size(); ++i) {insert(ts[i]);}
    types.insert(types.end(), ts.types.begin(), ts.types.end());
  }

  unsigned size() const {return tokens.size();}

  T & operator[](std::size_t i) {return tokens[i];}

  const T & operator[](std::size_t i) const {return tokens[i];}

  void clear()
  {
    for (unsigned i = 0; i < size(); ++i) {delete tokens[i];}

    tokens.clear();
    tokenMap.clear();
  }

  unsigned insert(const T & t)
  {
    TokenMap::iterator i = tokenMap.insert(tokenMap.begin(), std::make_pair(getName(t), size()));
    tokens.push_back(t);
    return i->second;
  }

  int index(const std::string & s) const
  {
    TokenMap::const_iterator i = tokenMap.find(s);
    return i == tokenMap.end() ? -1 : i->second;
  }

  T get(const std::string & s) const
  {
    int i = index(s);
    if (i < 0) {
      throw std::runtime_error(s + " does not name a known token");
    }
    return tokens[i];
  }
};

// Throws if a name appears twice in a list read from PDDL
inline void assertUnique(const TokenStruct<std::string> & ts, const std::string & what)
{
  if (ts.tokenMap.size() == ts.size()) {
    return;
  }
  std::set<std::string> seen;
  for (unsigned i = 0; i < ts.size(); ++i) {
    if (!seen.insert(ts[i]).second) {
      throw std::runtime_error("Duplicate " + what + " \"" + ts[i] + "\"");
    }
  }
}

// The names in the scope of a quantifier body: those outside, then its variables, which
// hide outer ones with the same name
inline TokenStruct<std::string> quantifierScope(
  const TokenStruct<std::string> & outer, const TokenStruct<std::string> & vars)
{
  TokenStruct<std::string> scope(outer);
  for (unsigned i = 0; i < vars.size(); ++i) {
    scope.tokenMap[vars[i]] = scope.size();
    scope.tokens.push_back(vars[i]);
  }
  scope.types.insert(scope.types.end(), vars.types.begin(), vars.types.end());
  return scope;
}

// The same scope when printing: base names from outside, then the variables
inline TokenStruct<std::string> printScope(
  const TokenStruct<std::string> & outer, unsigned base, const std::vector<std::string> & names)
{
  TokenStruct<std::string> scope;
  for (unsigned i = 0; i < base; ++i) {
    scope.tokens.push_back(i < outer.size() ? outer[i] : "?" + std::to_string(i));
  }
  for (const auto & name : names) {
    scope.tokens.push_back(name);
  }
  for (unsigned i = 0; i < scope.size(); ++i) {
    scope.tokenMap[scope.tokens[i]] = i;
  }
  return scope;
}

}  // namespace pddl
}  // namespace parser
