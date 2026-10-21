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

#include "plansys2_executor/ConstraintMonitor.hpp"

#include <iostream>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "plansys2_pddl_parser/Domain.hpp"
#include "plansys2_pddl_parser/Instance.hpp"
#include "plansys2_pddl_parser/Utils.hpp"

namespace plansys2
{

namespace
{
using Node = plansys2_msgs::msg::Node;
using Tree = plansys2_msgs::msg::Tree;

// The subtree from id, as a tree of its own
Tree subtree(const Tree & tree, uint32_t id)
{
  return parser::pddl::fromString(parser::pddl::toString(tree, id));
}

// The objects of a type and of its subtypes
void objects_of(const parser::pddl::Type * type, std::set<std::string> & ret)
{
  for (const auto & name : type->constants.tokens) {
    ret.insert(name);
  }
  for (const auto & name : type->objects.tokens) {
    ret.insert(name);
  }
  for (const auto * sub : type->subtypes) {
    objects_of(sub, ret);
  }
}

// forall grounded over the objects of the problem: an and of the bodies, one per object
Tree ground(const Tree & tree, const parser::pddl::Domain & domain)
{
  auto text = parser::pddl::toString(tree);
  for (size_t guard = 0; guard < 64; guard++) {
    auto parsed = parser::pddl::fromString(text);
    std::optional<uint32_t> forall;
    for (const auto & node : parsed.nodes) {
      if (node.node_type == Node::FORALL) {
        forall = node.node_id;
        break;
      }
    }
    if (!forall) {
      return parsed;
    }
    const auto & node = parsed.nodes[*forall];
    std::string body = parser::pddl::toString(parsed, node.children[0]);
    std::vector<std::string> bodies = {body};
    for (const auto & param : node.parameters) {
      std::set<std::string> objects;
      int index = domain.types.index(param.type);
      if (index >= 0) {
        objects_of(domain.types[index], objects);
      }
      std::vector<std::string> next;
      for (const auto & b : bodies) {
        for (const auto & object : objects) {
          // The variable as a whole word: ?0 is not ?01
          std::string out;
          for (size_t i = 0; i < b.size(); ) {
            if (b.compare(i, param.name.size(), param.name) == 0) {
              const size_t end = i + param.name.size();
              if (end == b.size() || !(std::isalnum(static_cast<unsigned char>(b[end])) ||
                b[end] == '_' || b[end] == '-'))
              {
                out += object;
                i = end;
                continue;
              }
            }
            out += b[i++];
          }
          next.push_back(out);
        }
      }
      bodies = next;
    }
    std::string grounded = "(and";
    for (const auto & b : bodies) {
      grounded += " " + b;
    }
    grounded += ")";
    const auto whole = parser::pddl::toString(parsed, *forall);
    text.replace(text.find(whole), whole.size(), grounded);
  }
  std::cerr << "ConstraintMonitor: forall nested too deep" << std::endl;
  return parser::pddl::fromString(text);
}

Tree tree_of(
  const parser::pddl::Condition * cond, const parser::pddl::Domain & domain,
  const std::vector<std::string> & objects)
{
  Tree tree;
  if (cond) {
    cond->getTree(tree, domain, objects);
  }
  return tree;
}
}  // namespace

ConstraintMonitor::ConstraintMonitor(const std::string & domain, const std::string & problem)
{
  try {
    parser::pddl::Domain d(domain);
    parser::pddl::Instance p(d);
    p.parse(problem);
    for (const auto & tree : {tree_of(d.constraints, d, {}),
        tree_of(p.constraints, d, p.ts.tokens)})
    {
      if (!tree.nodes.empty()) {
        const auto grounded = ground(tree, d);
        if (!grounded.nodes.empty()) {
          add(grounded, 0);
        }
      }
    }
  } catch (const std::exception & e) {
    std::cerr << "ConstraintMonitor: the constraints cannot be read: " << e.what() << std::endl;
  }
}

ConstraintMonitor::ConstraintMonitor(const plansys2_msgs::msg::Tree & constraints)
{
  if (!constraints.nodes.empty()) {
    add(constraints, 0);
  }
}

void
ConstraintMonitor::add(const Tree & tree, uint32_t id)
{
  const auto & node = tree.nodes[id];
  switch (node.node_type) {
    case Node::AND:
      for (auto child : node.children) {
        add(tree, child);
      }
      break;
    case Node::PREFERENCE:
      break;  // For the planner only
    case Node::CONSTRAINT: {
        Constraint c;
        c.op = node.name;
        c.text = parser::pddl::toString(tree, id);
        for (auto child : node.children) {
          if (tree.nodes[child].node_type == Node::NUMBER && c.conds.empty() &&
            c.times.size() < parser::pddl::constraintArity(c.op).first)
          {
            c.times.push_back(tree.nodes[child].value);
          } else {
            c.conds.push_back(subtree(tree, child));
          }
        }
        c.last.assign(c.conds.size(), false);
        constraints_.push_back(c);
        break;
      }
    default:
      std::cerr << "ConstraintMonitor: not a constraint, left out: " <<
        parser::pddl::toString(tree, id) << std::endl;
  }
}

bool
ConstraintMonitor::timed() const
{
  for (const auto & c : constraints_) {
    if (!c.times.empty()) {
      return true;
    }
  }
  return false;
}

std::vector<std::string>
ConstraintMonitor::describe() const
{
  std::vector<std::string> ret;
  for (const auto & c : constraints_) {
    ret.push_back(c.text);
  }
  return ret;
}

std::optional<std::string>
ConstraintMonitor::update(double t, const Holds & holds)
{
  // Up to now, the state was the previous one
  if (auto violated = check_times(t)) {
    return violated;
  }
  for (auto & c : constraints_) {
    std::vector<bool> now;
    for (const auto & cond : c.conds) {
      now.push_back(holds(cond));
    }
    const bool phi = now[0];
    if (c.op == "always") {
      if (!phi) {return c.text;}
    } else if (c.op == "sometime" || c.op == "at end") {
      c.seen = c.seen || phi;
    } else if (c.op == "at-most-once") {
      if (phi && (!started_ || !c.last[0])) {
        c.intervals++;
      }
      if (c.intervals > 1) {return c.text;}
    } else if (c.op == "within") {
      if (phi && t <= c.times[0] + 1e-6) {
        c.seen = true;
      }
    } else if (c.op == "sometime-after") {
      // Whenever the first holds, the second then or later
      if (now[1]) {
        c.deadline.reset();
      } else if (phi) {
        c.deadline = 0.0;  // Pending, with no time limit
      }
    } else if (c.op == "sometime-before") {
      // The first only after the second held, in an earlier state
      if (phi && !c.seen_second) {return c.text;}
      c.seen_second = c.seen_second || now[1];
    } else if (c.op == "always-within") {
      if (now[1]) {
        c.deadline.reset();
      } else if (phi && !c.deadline) {
        c.deadline = t + c.times[0];
      }
    }
    c.last = now;
  }
  started_ = true;
  return check_times(t);
}

std::optional<std::string>
ConstraintMonitor::check_times(double t)
{
  for (auto & c : constraints_) {
    if (c.op == "within" && !c.seen && t > c.times[0] + 1e-6) {
      return c.text;
    }
    if (c.op == "always-within" && c.deadline && t > *c.deadline + 1e-6) {
      return c.text;
    }
    // The state is as it was since the last update
    if (!started_) {
      continue;
    }
    if (c.op == "hold-during" && t >= c.times[0] && t < c.times[1] && !c.last[0]) {
      return c.text;
    }
    if (c.op == "hold-after" && t > c.times[0] && !c.last[0]) {
      return c.text;
    }
  }
  return {};
}

std::optional<std::string>
ConstraintMonitor::tick(double t)
{
  return check_times(t);
}

std::optional<std::string>
ConstraintMonitor::finish(double t)
{
  if (auto violated = check_times(t)) {
    return violated;
  }
  for (const auto & c : constraints_) {
    if (c.op == "sometime" && !c.seen) {return c.text;}
    if (c.op == "at end" && !(started_ && c.last[0])) {return c.text;}
    if (c.op == "within" && !c.seen) {return c.text;}
    if ((c.op == "sometime-after" || c.op == "always-within") && c.deadline) {return c.text;}
    // A window not reached yet: it never was
    if (c.op == "hold-during" && t < c.times[0]) {return c.text;}
  }
  return {};
}

}  // namespace plansys2
