// Copyright 2020 Intelligent Robotics Lab
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

#include <algorithm>
#include <cctype>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <queue>
#include <set>
#include <sstream>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "plansys2_executor/bt_builder_plugins/stn_bt_builder.hpp"
#include "plansys2_problem_expert/Utils.hpp"

namespace plansys2
{

namespace
{
// The functions the continuous effects of an action change, grounded
std::set<std::string> continuous_functions(const ActionStamped & action)
{
  std::set<std::string> ret;
  if (action.type != ActionType::START && action.type != ActionType::END) {
    return ret;
  }
  const auto effects = action.action.get_continuous_effects();
  for (const auto & node : effects.nodes) {
    if (node.node_type == plansys2_msgs::msg::Node::FUNCTION_MODIFIER && !node.children.empty()) {
      ret.insert(parser::pddl::toString(effects, node.children[0]));
    }
  }
  return ret;
}

// The facts a tree reads or writes: grounded, "robot_at r1 hall"; by name, when some argument is
// a variable; all, when it reads a derived predicate, which may read anything
struct Facts
{
  std::set<std::string> grounded;
  std::set<std::string> names;
  bool all = false;
};

void collect_facts(
  const plansys2_msgs::msg::Tree & tree, const std::set<std::string> & derived, Facts & facts)
{
  for (const auto & node : tree.nodes) {
    if (node.node_type != plansys2_msgs::msg::Node::PREDICATE &&
      node.node_type != plansys2_msgs::msg::Node::FUNCTION)
    {
      continue;
    }
    if (derived.count(node.name)) {
      facts.all = true;
      return;
    }
    // PDDL is case insensitive: a condition and an effect may not write a name the same way
    auto lower = [](std::string text) {
        for (auto & c : text) {
          c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        }
        return text;
      };
    bool variable = false;
    const std::string name = lower(node.name);
    std::string key = name;
    for (const auto & param : node.parameters) {
      variable = variable || param.name.empty() || param.name[0] == '?';
      key += " " + lower(param.name);
    }
    (variable ? facts.names : facts.grounded).insert(variable ? name : key);
  }
}

// Whether what one writes is something the other reads or writes
bool meet(const Facts & a, const Facts & b)
{
  if (a.all || b.all) {
    return true;
  }
  auto name_of = [](const std::string & key) {return key.substr(0, key.find(' '));};
  for (const auto & key : a.grounded) {
    if (b.grounded.count(key) || b.names.count(name_of(key))) {
      return true;
    }
  }
  for (const auto & name : a.names) {
    if (b.names.count(name)) {
      return true;
    }
    for (const auto & key : b.grounded) {
      if (name_of(key) == name) {
        return true;
      }
    }
  }
  return false;
}

// Happenings with more snaps than this are not searched in every order: 2^n subsets
constexpr int kMaxSubsetSnaps = 6;

// Snaps of the same plan item: a plan may have the same action several times
bool same_item(const ActionStamped & a, const ActionStamped & b)
{
  return a.expression == b.expression && a.time == b.time;
}
}  // namespace

STNBTBuilder::STNBTBuilder()
{
}

void
STNBTBuilder::initialize(
  const std::string & bt_action_1,
  const std::string & bt_action_2,
  int precision)
{
  if (bt_action_1 != "") {
    bt_start_action_ = bt_action_1;
  } else {
    bt_start_action_ =
      R""""(<Sequence name="ACTION_ID">
WAIT_PREV_ACTIONS
  <WaitAtStartReq action="ACTION_ID"/>
  <ApplyAtStartEffect action="ACTION_ID"/>
</Sequence>
)"""";
  }

  if (bt_action_2 != "") {
    bt_end_action_ = bt_action_2;
  } else {
    bt_end_action_ =
      R""""(<Sequence name="ACTION_ID">
  <ReactiveSequence name="ACTION_ID">
  <CheckOverAllReq action="ACTION_ID"/>
    <ExecuteAction action="ACTION_ID"/>
  </ReactiveSequence>
CHECK_PREV_ACTIONS
  <CheckAtEndReq action="ACTION_ID"/>
  <ApplyAtEndEffect action="ACTION_ID"/>
</Sequence>
)"""";
  }

  action_time_precision_ = precision;
}

std::string
STNBTBuilder::get_tree(const plansys2_msgs::msg::Plan & plan)
{
  // A tree for a plan that cannot run from the current state would wait forever
  if (!is_executable(plan)) {
    return {};
  }

  stn_ = build_stn(plan);

  if (!propagate(stn_)) {
    return {};
  }

  auto bt = build_bt(stn_);
  return bt;
}

std::string
STNBTBuilder::get_dotgraph(
  std::shared_ptr<std::map<std::string, ActionExecutionInfo>> action_map,
  bool enable_legend,
  bool enable_print_graph)
{
  if (!stn_) {
    return "";  // No plan yet
  }
  if (enable_print_graph) {
    print_graph(stn_);
  }

  // create xdot graph
  std::stringstream ss;
  ss.setf(std::ios::fixed);
  ss.precision(2);
  ss << "digraph plan {\n";

  // dotgraph formatting options
  ss << t(1) << "node[shape=box];\n";
  ss << t(1) << "rankdir=TB;\n";

  int node_count = 0;
  for (const auto & node : stn_->nodes) {
    ss << t(1) << "subgraph cluster_" << node_count << " {\n";
    auto start_time = node->action.time;
    auto duration = node->action.duration;
    if (node->action.type == ActionType::END) {
      start_time += node->action.duration;
      duration = 0.0;
    }
    ss << t(2) << "label = \"Start: " << start_time << "\nDuration: " << duration << " s\";\n";
    ss << t(2) << "style = rounded;\n";
    ss << t(2) << "color = yellow3;\n";
    ss << t(2) << "bgcolor = lemonchiffon;\n";
    ss << t(2) << "labeljust = l;\n";
    ss << get_node_dotgraph(node, action_map);
    ss << t(1) << "}\n";
    node_count++;
  }

  // define the edges
  std::set<std::string> edges;
  std::set<const Node *> visited;
  for (const auto & graph_root : stn_->nodes) {
    get_flow_dotgraph(graph_root, edges, visited);
  }

  for (const auto & edge : edges) {
    ss << t(1) << edge;
  }

  if (enable_legend) {
    ss << add_dot_graph_legend(node_count, node_count);
  }

  ss << "}";

  return ss.str();
}

bool
STNBTBuilder::is_executable(const plansys2_msgs::msg::Plan & plan) const
{
  reset_derived_rules();  // The instances of now
  auto happenings = get_happenings(plan);
  auto simple_plan = get_simple_plan(plan);
  auto states = get_states(happenings, simple_plan);

  // Every snap action must be applicable at its happening, as get_threat assumes
  for (const auto & item : simple_plan) {
    if (item.first < 0) {
      continue;  // The initial state
    }
    auto happening = get_happening(item.first, happenings);
    auto previous = happening == happenings.end() ?
      happenings.end() : get_previous(*happening, happenings);
    if (previous == happenings.end() || states.find(*previous) == states.end()) {
      return false;
    }
    auto state = states.at(*previous);
    if (!can_apply(item, simple_plan, *happening, state)) {
      // PDDL+: the dynamics give it while the plan runs; its tree waits for it
      if (reads_dynamic(get_conditions(item.second))) {
        continue;
      }
      std::cerr << "STNBTBuilder: " << item.second.expression << " (" <<
        to_string(item.second.type) << ") cannot be applied: the plan cannot be executed" <<
        std::endl;
      return false;
    }
  }
  return true;
}

Graph::Ptr
STNBTBuilder::build_stn(const plansys2_msgs::msg::Plan & plan) const
{
  auto stn = init_graph(plan);
  auto happenings = get_happenings(plan);
  auto simple_plan = get_simple_plan(plan);
  auto states = get_states(happenings, simple_plan);

  for (const auto & item : simple_plan) {
    // Skip the first action corresponding to the initial state
    if (item.first < 0) {
      continue;
    }

    // Get the graph nodes corresponding to the current action
    auto current = get_nodes(item.second, stn);

    // Get the parent actions of the current action
    auto parents = get_parents(item, simple_plan, happenings, states);

    for (const auto & parent : parents) {
      // A non-durative action runs whole in its performer: what follows waits for its end
      auto after = parent.second;
      if (after.type == ActionType::START && after.action.is_action()) {
        after.type = ActionType::END;
      }
      auto previous = get_nodes(after, stn);
      for (auto & n : current) {
        for (auto & h : previous) {
          if (h->action.time == n->action.time) {
            if (h->action.expression == n->action.expression) {
              // No self-referencing edges are allowed in an STN.
              continue;
            }
          }
          prune_paths(n, h);
          if (!check_paths(n, h)) {
            h->output_arcs.insert(std::make_tuple(n, 0.0, std::numeric_limits<double>::infinity()));
            n->input_arcs.insert(std::make_tuple(h, 0.0, std::numeric_limits<double>::infinity()));
          }
        }
      }
    }
  }

  return stn;
}

bool
STNBTBuilder::propagate(const Graph::Ptr stn)
{
  // Compute the distance matrix.
  Eigen::MatrixXd dist = get_distance_matrix(stn);

  // Check if STN is consistent.
  for (Eigen::Index i = 0; i < dist.rows(); i++) {
    if (dist(i, i) < 0) {
      return false;
    }
  }

  // Update the STN.
  for (auto node : stn->nodes) {
    int row = node->node_num;

    // Create a set to hold the updated output arcs.
    std::set<std::tuple<Node::Ptr, double, double>> output_arcs;

    // Iterate over the output arcs.
    for (auto arc_out : node->output_arcs) {
      auto child = std::get<0>(arc_out);
      auto col = child->node_num;

      // Get the new lower and upper bounds.
      auto upper = dist(row, col);
      auto lower = -dist(col, row);

      // Save the updated output arc.
      output_arcs.insert(std::make_tuple(child, lower, upper));

      // Find the child input arc.
      auto it = std::find_if(
        child->input_arcs.begin(), child->input_arcs.end(),
        [&](std::tuple<Node::Ptr, double, double> arc_in) {
          return std::get<0>(arc_in) == node;
        });

      if (it != child->input_arcs.end()) {
        child->input_arcs.erase(it);
      }
      child->input_arcs.insert(std::make_tuple(node, lower, upper));
    }

    // Replace the output arcs.
    node->output_arcs.clear();
    node->output_arcs = output_arcs;
  }

  return true;
}

std::string
STNBTBuilder::build_bt(const Graph::Ptr stn) const
{
  std::set<Node::Ptr> used;
  const auto & root = stn->nodes.front();

  auto bt = std::string("<root BTCPP_format=\"4\" main_tree_to_execute=\"MainTree\">\n") + t(1) +
    "<BehaviorTree ID=\"MainTree\">\n";
  get_flow(root, nullptr, used, 1, true, bt);
  bt = bt + t(1) + "</BehaviorTree>\n</root>\n";

  return bt;
}

Graph::Ptr
STNBTBuilder::init_graph(const plansys2_msgs::msg::Plan & plan) const
{
  auto graph = Graph::make_shared();
  graph->action_time_precision = action_time_precision_;
  auto action_sequence = get_plan_actions(plan);

  // Add a node to represent the initial state
  auto predicates = initial_predicates();
  auto functions = initial_functions();

  auto init_action = std::make_shared<plansys2_msgs::msg::DurativeAction>();
  init_action->at_end_effects = from_state(predicates, functions);

  int node_cnt = 0;
  auto init_node = Node::make_shared(node_cnt++);
  init_node->action.action = init_action;
  init_node->action.type = ActionType::INIT;
  graph->nodes.push_back(init_node);

  // Add nodes for the start and end snap actions
  for (const auto & action : action_sequence) {
    auto start_node = Node::make_shared(node_cnt++);
    start_node->action = action;
    start_node->action.type = ActionType::START;

    auto end_node = Node::make_shared(node_cnt++);
    end_node->action = action;
    end_node->action.type = ActionType::END;

    start_node->output_arcs.insert(std::make_tuple(end_node, action.duration, action.duration));
    end_node->input_arcs.insert(std::make_tuple(start_node, action.duration, action.duration));

    graph->nodes.push_back(start_node);
    graph->nodes.push_back(end_node);
  }

  // Add a node to represent the goal
  auto goal = problem_client()->getGoal();
  plansys2_msgs::msg::Tree * goal_tree = &goal;

  auto goal_action = std::make_shared<plansys2_msgs::msg::DurativeAction>();
  goal_action->at_start_requirements = *goal_tree;
  auto goal_node = Node::make_shared(node_cnt++);
  goal_node->action.action = goal_action;
  goal_node->action.type = ActionType::GOAL;
  graph->nodes.push_back(goal_node);

  return graph;
}

std::vector<ActionStamped>
STNBTBuilder::get_plan_actions(const plansys2_msgs::msg::Plan & plan) const
{
  if (plan_actions_ && plan_actions_->first == plan) {
    return plan_actions_->second;
  }
  std::vector<ActionStamped> ret;

  // Once for the whole plan: it was asked for every action
  const auto actions = domain_client()->getActions();
  for (auto & item : plan.items) {
    ActionStamped action_stamped;
    action_stamped.time = item.time;
    action_stamped.expression = item.action;
    action_stamped.duration = item.duration;
    action_stamped.type = ActionType::DURATIVE;
    action_stamped.action = plan_item_action(*domain_client(), actions, item);

    ret.push_back(action_stamped);
  }

  plan_actions_.emplace(plan, ret);
  return ret;
}

std::set<int64_t>
STNBTBuilder::get_happenings(const plansys2_msgs::msg::Plan & plan) const
{
  std::set<int64_t> happenings;
  happenings.insert(-1);
  auto action_sequence = get_plan_actions(plan);
  for (const auto & action : action_sequence) {
    auto time = to_int_time(action.time, action_time_precision_ + 1);
    auto duration = to_int_time(action.duration, action_time_precision_ + 1);
    happenings.insert(time);
    happenings.insert(time + duration);
  }
  // The timed facts happen too
  for (const auto & event : timed_events()) {
    happenings.insert(to_int_time(event.time, action_time_precision_ + 1));
  }
  return happenings;
}

std::set<int64_t>::iterator
STNBTBuilder::get_happening(int64_t time, const std::set<int64_t> & happenings) const
{
  // This function returns an iterator pointing to either
  //   1. the first element that is equal to the key or
  //   2. the last element that is less than the key,
  // whichever of the two is greater.

  // lower_bound returns an iterator pointing to the first element that is
  // not less than (i.e. greater or equal to) key.
  auto it = happenings.lower_bound(time);
  if (it != happenings.end()) {
    if (*it != time) {
      if (it != happenings.begin()) {
        it--;
      } else {
        it = happenings.end();
      }
    }
  }

  return it;
}

std::set<int64_t>::iterator
STNBTBuilder::get_previous(int64_t time, const std::set<int64_t> & happenings) const
{
  auto it = get_happening(time, happenings);

  if (it != happenings.end()) {
    if (it != happenings.begin()) {
      return std::prev(it);
    }
  }

  return happenings.end();
}

std::multimap<int64_t, ActionStamped>
STNBTBuilder::get_simple_plan(const plansys2_msgs::msg::Plan & plan) const
{
  std::multimap<int64_t, ActionStamped> simple_plan;
  auto action_sequence = get_plan_actions(plan);

  // Add an action to represent the initial state
  auto predicates = initial_predicates();
  auto functions = initial_functions();

  auto init_action_ = std::make_shared<plansys2_msgs::msg::DurativeAction>();
  init_action_->at_end_effects = from_state(predicates, functions);
  ActionStamped init_action;
  init_action.action = init_action_;
  init_action.type = ActionType::INIT;
  simple_plan.insert(std::make_pair(-1, init_action));

  // Add the snap actions
  int64_t max_time = -1;
  for (auto action : action_sequence) {
    auto time = to_int_time(action.time, action_time_precision_ + 1);
    auto duration = to_int_time(action.duration, action_time_precision_ + 1);
    action.type = ActionType::START;
    simple_plan.insert(std::make_pair(time, action));
    action.type = ActionType::END;
    simple_plan.insert(std::make_pair(time + duration, action));
    max_time = std::max(max_time, time + duration);
  }

  // Add an action to represent the goal
  auto goal = problem_client()->getGoal();
  plansys2_msgs::msg::Tree * goal_tree = &goal;

  auto goal_action_ = std::make_shared<plansys2_msgs::msg::DurativeAction>();
  goal_action_->at_start_requirements = *goal_tree;

  ActionStamped goal_action;
  goal_action.action = goal_action_;
  goal_action.type = ActionType::GOAL;
  simple_plan.insert(std::make_pair(max_time, goal_action));

  // The timed facts, at their time: only effects, which the problem expert applies
  for (const auto & event : timed_events()) {
    simple_plan.insert(
      std::make_pair(to_int_time(event.time, action_time_precision_ + 1), event));
  }

  // Create the overall actions
  std::vector<std::pair<int64_t, ActionStamped>> overall_actions;
  for (const auto & action : action_sequence) {
    auto time = to_int_time(action.time, action_time_precision_ + 1);
    auto duration = to_int_time(action.duration, action_time_precision_ + 1);

    // Find the start action
    auto it = simple_plan.equal_range(time);
    auto start_action = std::find_if(
      it.first, it.second,
      [&](std::pair<int64_t, ActionStamped> a) {
        return same_item(a.second, action) && (a.second.type == ActionType::START);
      });

    // Find the end action
    it = simple_plan.equal_range(time + duration);
    auto end_action = std::find_if(
      it.first, it.second,
      [&](std::pair<int64_t, ActionStamped> a) {
        return same_item(a.second, action) && (a.second.type == ActionType::END);
      });

    // An action with no duration (a non-durative one, as some planners give it) has no
    // interval: its overall conditions hold at its only happening
    if (duration == 0) {
      auto overall_action = std::make_pair(time, start_action->second);
      overall_action.second.type = ActionType::OVERALL;
      overall_actions.push_back(overall_action);
      continue;
    }

    // Compute the overall actions
    int64_t prev = time;
    for (auto iter = start_action; iter != simple_plan.end(); ++iter) {
      if (iter->first != prev) {
        int64_t time = prev + (iter->first - prev) / 2;
        auto overall_action = std::make_pair(time, start_action->second);
        overall_action.second.type = ActionType::OVERALL;
        overall_actions.push_back(overall_action);
        prev = iter->first;
      }
      if (iter == end_action) {
        break;
      }
    }
  }

  // Add the overall actions
  for (const auto & overall_action : overall_actions) {
    simple_plan.insert(overall_action);
  }

  return simple_plan;
}

std::map<int64_t, StateVec>
STNBTBuilder::get_states(
  const std::set<int64_t> & happenings,
  const std::multimap<int64_t, ActionStamped> & plan) const
{
  std::map<int64_t, StateVec> states;

  StateVec state_vec;
  state_vec.predicates = initial_predicates();
  state_vec.functions = initial_functions();
  states.insert(std::make_pair(-1, state_vec));

  for (const auto & time : happenings) {
    auto it = plan.equal_range(time);
    for (auto iter = it.first; iter != it.second; ++iter) {
      if (iter->second.type == ActionType::START) {
        apply_effects(
          iter->second.action.get_at_start_effects(), state_vec.predicates, state_vec.functions);
      } else if (iter->second.type == ActionType::END || iter->second.type == ActionType::TIMED) {
        apply_effects(
          iter->second.action.get_at_end_effects(), state_vec.predicates, state_vec.functions);
      }
    }
    states.insert(std::make_pair(time, state_vec));
  }

  return states;
}

plansys2_msgs::msg::Tree
STNBTBuilder::from_state(
  const std::vector<plansys2::Predicate> & preds,
  const std::vector<plansys2::Function> & funcs) const
{
  plansys2_msgs::msg::Tree tree;
  plansys2_msgs::msg::Node node;
  node.node_type = plansys2_msgs::msg::Node::AND;
  node.node_id = 0;
  node.negate = false;
  tree.nodes.push_back(node);

  for (const auto & pred : preds) {
    const plansys2_msgs::msg::Node * child = &pred;
    tree.nodes.push_back(*child);
    tree.nodes.back().node_id = tree.nodes.size() - 1;
    tree.nodes[0].children.push_back(tree.nodes.size() - 1);
  }

  for (const auto & func : funcs) {
    const plansys2_msgs::msg::Node * child = &func;
    tree.nodes.push_back(*child);
    tree.nodes.back().node_id = tree.nodes.size() - 1;
    tree.nodes[0].children.push_back(tree.nodes.size() - 1);
  }

  return tree;
}

std::vector<Node::Ptr>
STNBTBuilder::get_nodes(
  const ActionStamped & action,
  const Graph::Ptr graph) const
{
  std::vector<Node::Ptr> ret;

  // A timed fact has no node: what needs it waits in its tree until the problem expert has it
  if (action.type == ActionType::TIMED) {
    return ret;
  }

  if (action.type == ActionType::INIT) {
    auto it = std::find_if(
      graph->nodes.begin(), graph->nodes.end(), [&](Node::Ptr node) {
        return node->action.type == ActionType::INIT;
      });
    if (it != graph->nodes.end()) {
      ret.push_back(*it);
    } else {
      std::cerr << "get_nodes: Could not find initial state node" << std::endl;
    }
    return ret;
  }

  if (action.type == ActionType::GOAL) {
    auto it = std::find_if(
      graph->nodes.begin(), graph->nodes.end(), [&](Node::Ptr node) {
        return node->action.type == ActionType::GOAL;
      });
    if (it != graph->nodes.end()) {
      ret.push_back(*it);
    } else {
      std::cerr << "get_nodes: Could not find goal node" << std::endl;
    }
    return ret;
  }

  std::vector<Node::Ptr> matches;
  std::copy_if(
    graph->nodes.begin(), graph->nodes.end(), std::back_inserter(matches),
    std::bind(&STNBTBuilder::is_match, this, std::placeholders::_1, action));

  if (action.type == ActionType::START || action.type == ActionType::OVERALL) {
    auto it = std::find_if(
      matches.begin(), matches.end(), [&](Node::Ptr node) {
        return node->action.type == ActionType::START;
      });
    if (it != matches.end()) {
      ret.push_back(*it);
    } else {
      std::cerr << "get_nodes: Could not find start node" << std::endl;
    }
  }

  if (action.type == ActionType::END || action.type == ActionType::OVERALL) {
    auto it = std::find_if(
      matches.begin(), matches.end(), [&](Node::Ptr node) {
        return node->action.type == ActionType::END;
      });
    if (it != matches.end()) {
      ret.push_back(*it);
    } else {
      std::cerr << "get_nodes: Could not find end node" << std::endl;
    }
  }

  if (ret.empty()) {
    std::cerr << "get_nodes: Could not find graph node" << std::endl;
  }

  return ret;
}

bool
STNBTBuilder::is_match(
  const Node::Ptr node,
  const ActionStamped & action) const
{
  auto t_1 = to_int_time(node->action.time, action_time_precision_ + 1);
  auto t_2 = to_int_time(action.time, action_time_precision_ + 1);
  return (t_1 == t_2) && (node->action.expression == action.expression);
}

std::vector<std::pair<int64_t, ActionStamped>>
STNBTBuilder::get_parents(
  const std::pair<int64_t, ActionStamped> & action,
  const std::multimap<int64_t, ActionStamped> & plan,
  const std::set<int64_t> & happenings,
  const std::map<int64_t, StateVec> & states) const
{
  auto parents = get_satisfy(action, plan, happenings, states);
  auto threats = get_threat(action, plan, happenings, states);
  parents.insert(std::end(parents), std::begin(threats), std::end(threats));

  return parents;
}

std::vector<std::pair<int64_t, ActionStamped>>
STNBTBuilder::get_satisfy(
  const std::pair<int64_t, ActionStamped> & action,
  const std::multimap<int64_t, ActionStamped> & plan,
  const std::set<int64_t> & happenings,
  const std::map<int64_t, StateVec> & states) const
{
  std::vector<std::pair<int64_t, ActionStamped>> ret;

  auto H_it = get_happening(action.first, happenings);
  if (H_it == happenings.end()) {
    std::cerr << "get_satisfy: Happening time not found" << std::endl;
    return ret;
  }
  auto t_2 = *H_it;

  auto R_a = parser::pddl::getSubtrees(get_ordering_conditions(action, happenings, states));

  // What each condition reads: an action that writes none of it cannot make it true
  const auto & derived = derived_names();
  std::vector<Facts> reads_r(R_a.size());
  for (size_t i = 0; i < R_a.size(); i++) {
    collect_facts(R_a[i], derived, reads_r[i]);
  }

  while (t_2 >= 0) {
    H_it = get_previous(t_2, happenings);
    if (H_it == happenings.end()) {
      std::cerr << "get_satisfy: Previous happening time not found" << std::endl;
      break;
    }
    auto t_1 = *H_it;

    auto X_it = states.find(t_1);
    if (X_it == states.end()) {
      std::cerr << "get_satisfy: Previous state not found" << std::endl;
      break;
    }
    // Not copied: a long plan has many states, and big. check() only reads it (it evaluates
    // without applying), though it takes it as non-const
    auto & X_1 = const_cast<StateVec &>(X_it->second);

    for (size_t i = 0; i < R_a.size(); i++) {
      const auto & r = R_a[i];
      if (!check(r, X_1.predicates, X_1.functions)) {
        auto it = plan.equal_range(t_2);
        for (auto iter = it.first; iter != it.second; ++iter) {
          if (iter->first == action.first) {
            if (same_item(iter->second, action.second)) {
              continue;
            }
          }

          auto E_k = get_effects(iter->second);
          Facts writes_k;
          collect_facts(E_k, derived, writes_k);
          if (!meet(writes_k, reads_r[i])) {
            continue;
          }

          auto X_hat = X_1;
          apply_effects(E_k, X_hat.predicates, X_hat.functions);

          // Check if action k satisfies action i
          if (check(r, X_hat.predicates, X_hat.functions)) {
            ret.push_back(*iter);
          }
        }
      }
    }
    t_2 = t_1;
  }

  auto predicates = initial_predicates();
  auto functions = initial_functions();

  for (const auto & r : R_a) {
    if (check(r, predicates, functions)) {
      ret.push_back(*plan.begin());
    }
  }

  return ret;
}

std::vector<std::pair<int64_t, ActionStamped>>
STNBTBuilder::get_threat(
  const std::pair<int64_t, ActionStamped> & action,
  const std::multimap<int64_t, ActionStamped> & plan,
  const std::set<int64_t> & happenings,
  const std::map<int64_t, StateVec> & states) const
{
  std::vector<std::pair<int64_t, ActionStamped>> ret;

  auto H_it = get_happening(action.first, happenings);
  if (H_it == happenings.end()) {
    std::cerr << "get_threat: Happening time not found" << std::endl;
    return ret;
  }
  auto t_in = *H_it;
  auto t_2 = *H_it;

  auto R_a = get_ordering_conditions(action, happenings, states);
  auto E_a = get_threat_effects(action.second);
  const auto rates_a = continuous_functions(action.second);

  // What each one reads and writes: those that share nothing cannot threaten each other
  const auto & derived = derived_names();
  auto facts = [&](const ActionStamped & snap) {
      std::pair<Facts, Facts> ret;  // Read, written
      collect_facts(get_conditions(snap), derived, ret.first);
      collect_facts(get_effects(snap), derived, ret.first);  // The conditions of a when too
      collect_facts(get_threat_effects(snap), derived, ret.second);
      return ret;
    };
  const auto [reads_a, writes_a] = facts(action.second);

  while (t_2 >= 0) {
    H_it = get_previous(t_2, happenings);
    if (H_it == happenings.end()) {
      std::cerr << "get_threat: Previous happening time not found" << std::endl;
      break;
    }
    auto t_1 = *H_it;

    auto X_it = states.find(t_1);
    if (X_it == states.end()) {
      std::cerr << "get_threat: Previous state not found" << std::endl;
      break;
    }

    auto it = plan.end();
    if (auto next_happening = std::next(happenings.find(t_2)); next_happening != happenings.end()) {
      it = plan.lower_bound(*next_happening);
    }

    // Nothing at this happening shares a fact with the action: no state to look at. Most of
    // them in a long plan, which made each action copy every state before it
    bool related = false;
    for (auto iter = plan.lower_bound(t_2); iter != it && !related; ++iter) {
      if (iter->first == action.first && same_item(iter->second, action.second)) {
        continue;
      }
      if (!continuous_functions(iter->second).empty() && !rates_a.empty()) {
        related = true;
        continue;
      }
      const auto [reads_k, writes_k] = facts(iter->second);
      related = meet(writes_a, reads_k) || meet(writes_a, writes_k) || meet(writes_k, reads_a);
    }
    if (!related) {
      t_2 = t_1;
      continue;
    }

    const auto & X_1 = X_it->second;
    auto X_1_a = X_1;
    if (can_apply(action, plan, t_2, X_1_a)) {
      // At its own happening, the plan order of the snap actions orders them
      bool after_self = false;
      auto order = [&](const std::pair<int64_t, ActionStamped> & other) {
          if (t_2 != t_in || !after_self) {
            ret.push_back(other);
          }
        };
      for (auto iter = plan.lower_bound(t_2); iter != it; ++iter) {
        if (iter->first == action.first) {
          if (same_item(iter->second, action.second)) {
            after_self = after_self || t_2 == t_in;
            continue;
          }
        }

        // Two rates on the same function add up: they keep the order of the plan
        const auto rates_k = continuous_functions(iter->second);
        if (rates_k.empty() || rates_a.empty()) {
          const auto [reads_k, writes_k] = facts(iter->second);
          if (!meet(writes_a, reads_k) && !meet(writes_a, writes_k) && !meet(writes_k, reads_a)) {
            continue;
          }
        }
        if (std::any_of(
            rates_a.begin(), rates_a.end(), [&](const auto & f) {return rates_k.count(f) > 0;}))
        {
          order(*iter);
          continue;
        }

        auto X_1_k = X_1;
        if (!can_apply(*iter, plan, t_2, X_1_k)) {
          // PDDL+: what the dynamics give, an event or a process, comes later in the plan
          if (!reads_dynamic(get_conditions(iter->second)) &&
            (iter->second.type != ActionType::GOAL ||
            !reads_dynamic(iter->second.action.get_at_start_requirements())))
          {
            std::cerr << "get_threat: Suitable intermediate state not found. ";
            std::cerr << "However, one should exist." << std::endl;
          }
          continue;
        }

        auto R_k = get_ordering_conditions(*iter, happenings, states);
        auto E_k = get_threat_effects(iter->second);

        auto X_hat = X_1_k;
        apply_effects(E_a, X_hat.predicates, X_hat.functions);

        // Check if the input action threatens action k
        if (action.second.type != ActionType::OVERALL &&
          !check(R_k, X_hat.predicates, X_hat.functions))
        {
          order(*iter);
          continue;
        }

        auto X_bar = X_1_a;
        apply_effects(E_k, X_bar.predicates, X_bar.functions);

        // Check if action k threatens the input action
        if (iter->second.type != ActionType::OVERALL &&
          !check(R_a, X_bar.predicates, X_bar.functions))
        {
          order(*iter);
          continue;
        }

        // Check if the input action and action k modify the same effect: one adds what the other
        // deletes, even if it already held, or both change it
        if (action.second.type != ActionType::OVERALL &&
          iter->second.type != ActionType::OVERALL)
        {
          auto DX_hat = get_diff(X_1_k, X_hat);
          auto DX_bar = get_diff(X_1_a, X_bar);
          auto intersection = get_intersection(DX_hat, DX_bar);
          if (intersection.predicates.size() > 0 || intersection.functions.size() > 0 ||
            interfere(E_a, E_k))
          {
            order(*iter);
          }
        }
      }
    }
    t_2 = t_1;
  }

  return ret;
}

bool
STNBTBuilder::can_apply(
  const std::pair<int64_t, ActionStamped> & action,
  const std::multimap<int64_t, ActionStamped> & plan,
  const int64_t & time,
  StateVec & state) const
{
  auto X = state;
  auto R = get_conditions(action.second);

  if (check(R, X.predicates, X.functions)) {
    return true;
  }

  // The snaps before it at its happening, in the order of the plan
  auto range = plan.equal_range(time);
  for (auto iter = range.first; iter != range.second; ++iter) {
    if (iter->first == action.first && same_item(iter->second, action.second)) {
      break;
    }
    apply_effects(get_effects(iter->second), state.predicates, state.functions);
    if (check(R, state.predicates, state.functions)) {
      return true;
    }
  }

  // Others in any order: every subset, so only for a few at once
  int n = plan.count(time);
  if (n > kMaxSubsetSnaps) {
    state = X;
    return false;
  }
  auto it = plan.lower_bound(time);
  for (int m = 1; m <= n; ++m) {
    std::vector<bool> v(n);
    std::fill(v.begin(), v.begin() + m, true);

    state = X;
    do {
      for (int i = 0; i < n; ++i) {
        if (v[i]) {
          auto iter = std::next(it, i);
          if (iter->first == action.first) {
            if (same_item(iter->second, action.second)) {
              continue;
            }
          }
          auto E = get_effects(iter->second);
          apply_effects(E, state.predicates, state.functions);
          if (check(R, state.predicates, state.functions)) {
            return true;
          }
        }
      }
    } while (std::prev_permutation(v.begin(), v.end()));
  }

  return false;
}

namespace
{

// What makes two facts the same for checkNodeEquality: the name and arguments, whatever the case
std::string fact_key(const plansys2_msgs::msg::Node & node)
{
  std::string key = std::to_string(node.node_type) + ' ';
  auto add = [&key](const std::string & text) {
      for (char c : text) {
        key += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
      }
    };
  add(node.name);
  for (const auto & param : node.parameters) {
    key += ' ';
    add(param.name);
  }
  return key;
}

bool has_variables(const plansys2_msgs::msg::Node & node)
{
  return std::any_of(
    node.parameters.begin(), node.parameters.end(),
    [](const plansys2_msgs::msg::Param & p) {return !p.name.empty() && p.name.front() == '?';});
}

// The facts of a state, found by key instead of one by one: a state of a long plan has many, and
// it is compared with many others. A fact with variables, which matches any argument there, is
// still compared one by one
template<typename NodeT>
class FactIndex
{
public:
  explicit FactIndex(const std::vector<NodeT> & facts)
  : facts_(facts)
  {
    for (const auto & fact : facts) {
      if (has_variables(fact)) {
        with_variables_.push_back(&fact);
      } else {
        by_key_.emplace(fact_key(fact), &fact);
      }
    }
  }

  const NodeT * find(const NodeT & fact) const
  {
    if (has_variables(fact)) {
      for (const auto & other : facts_) {
        if (parser::pddl::checkNodeEquality(fact, other)) {
          return &other;
        }
      }
      return nullptr;
    }
    if (auto it = by_key_.find(fact_key(fact)); it != by_key_.end()) {
      return it->second;
    }
    for (const auto * other : with_variables_) {
      if (parser::pddl::checkNodeEquality(fact, *other)) {
        return other;
      }
    }
    return nullptr;
  }

private:
  const std::vector<NodeT> & facts_;
  std::unordered_map<std::string, const NodeT *> by_key_;
  std::vector<const NodeT *> with_variables_;
};

}  // namespace

StateVec
STNBTBuilder::get_diff(
  const StateVec & X_1,
  const StateVec & X_2) const
{
  StateVec ret;
  const FactIndex<plansys2::Predicate> predicates_1(X_1.predicates);
  const FactIndex<plansys2::Predicate> predicates_2(X_2.predicates);

  // Predicates in X_1 that are not in X_2, and in X_2 that are not in X_1
  for (const auto & p_1 : X_1.predicates) {
    if (!predicates_2.find(p_1)) {
      ret.predicates.push_back(p_1);
    }
  }
  for (const auto & p_2 : X_2.predicates) {
    if (!predicates_1.find(p_2)) {
      ret.predicates.push_back(p_2);
    }
  }

  // Function changes
  const FactIndex<plansys2::Function> functions_2(X_2.functions);
  for (const auto & f_1 : X_1.functions) {
    if (const auto * f_2 = functions_2.find(f_1)) {
      if (std::abs(f_1.value - f_2->value) >
        1e-5 * std::max(std::abs(f_1.value), std::abs(f_2->value)))
      {
        ret.functions.push_back(f_1);
      }
    }
  }

  return ret;
}

StateVec
STNBTBuilder::get_intersection(
  const StateVec & X_1,
  const StateVec & X_2) const
{
  StateVec ret;

  // Predicates and functions in X_1 that are also in X_2
  const FactIndex<plansys2::Predicate> predicates_2(X_2.predicates);
  for (const auto & p_1 : X_1.predicates) {
    if (predicates_2.find(p_1)) {
      ret.predicates.push_back(p_1);
    }
  }
  const FactIndex<plansys2::Function> functions_2(X_2.functions);
  for (const auto & f_1 : X_1.functions) {
    if (functions_2.find(f_1)) {
      ret.functions.push_back(f_1);
    }
  }

  return ret;
}

plansys2_msgs::msg::Tree
STNBTBuilder::get_conditions(const ActionStamped & action) const
{
  // PDDL+: a discrete simulation cannot tell what the dynamics change; the tree waits for it
  // A non-durative action requires its precondition when it starts: otherwise nothing orders it
  // after what makes it true, and it starts too early
  if (action.type == ActionType::START && action.action.is_action()) {
    return relax_dynamic(action.action.get_overall_requirements());
  }
  if (action.type == ActionType::START || action.type == ActionType::GOAL) {
    return relax_dynamic(action.action.get_at_start_requirements());
  } else if (action.type == ActionType::OVERALL) {
    return relax_dynamic(action.action.get_overall_requirements());
  } else if (action.type == ActionType::END) {
    return relax_dynamic(action.action.get_at_end_requirements());
  }

  return plansys2_msgs::msg::Tree();
}

plansys2_msgs::msg::Tree
STNBTBuilder::get_ordering_conditions(
  const std::pair<int64_t, ActionStamped> & action, const std::set<int64_t> & happenings,
  const std::map<int64_t, StateVec> & states) const
{
  const auto conditions = get_conditions(action.second);
  const auto effects = get_effects(action.second);

  // The conditional effects, outside quantifiers
  std::vector<uint32_t> whens;
  std::vector<uint32_t> pending;
  if (!effects.nodes.empty()) {
    pending.push_back(0);
  }
  while (!pending.empty()) {
    const auto id = pending.back();
    pending.pop_back();
    const auto & node = effects.nodes[id];
    if (node.node_type == plansys2_msgs::msg::Node::AND) {
      pending.insert(pending.end(), node.children.begin(), node.children.end());
    } else if (node.node_type == plansys2_msgs::msg::Node::WHEN) {
      whens.push_back(id);
    }
  }
  if (whens.empty()) {
    return conditions;
  }

  // The state before its happening, as the plan has it
  StateVec state{initial_predicates(), initial_functions()};
  auto happening = get_happening(action.first, happenings);
  if (happening != happenings.end()) {
    auto previous = get_previous(*happening, happenings);
    if (previous != happenings.end() && states.count(*previous)) {
      state = states.at(*previous);
    }
  }

  std::string text = "(and";
  if (!conditions.nodes.empty()) {
    if (conditions.nodes[0].node_type == plansys2_msgs::msg::Node::AND) {
      for (auto child : conditions.nodes[0].children) {
        text += " " + parser::pddl::toString(conditions, child);
      }
    } else {
      text += " " + parser::pddl::toString(conditions);
    }
  }
  for (auto id : whens) {
    const auto condition = parser::pddl::toString(effects, effects.nodes[id].children[0]);
    const bool held = check(parser::pddl::fromString(condition), state.predicates, state.functions);
    text += held ? " " + condition : " (not " + condition + ")";
  }
  return parser::pddl::fromString(text + ")");
}

namespace
{
// The facts an effect adds and deletes, and the functions it changes, as written: conditional
// effects as if they happened
void written_effects(
  const plansys2_msgs::msg::Tree & tree, uint32_t id, bool negate,
  std::vector<plansys2_msgs::msg::Node> & adds, std::vector<plansys2_msgs::msg::Node> & deletes,
  std::vector<plansys2_msgs::msg::Node> & functions)
{
  if (id >= tree.nodes.size()) {
    return;
  }
  const auto & node = tree.nodes[id];
  switch (node.node_type) {
    case plansys2_msgs::msg::Node::AND:
      for (auto child : node.children) {
        written_effects(tree, child, negate, adds, deletes, functions);
      }
      break;
    case plansys2_msgs::msg::Node::NOT:
      written_effects(tree, node.children[0], !negate, adds, deletes, functions);
      break;
    case plansys2_msgs::msg::Node::WHEN:
      written_effects(tree, node.children[1], negate, adds, deletes, functions);
      break;
    case plansys2_msgs::msg::Node::PREDICATE:
      (negate ? deletes : adds).push_back(node);
      break;
    case plansys2_msgs::msg::Node::FUNCTION_MODIFIER:
      if (!node.children.empty()) {
        functions.push_back(tree.nodes[node.children[0]]);
      }
      break;
    default:
      break;
  }
}

bool any_equal(
  const std::vector<plansys2_msgs::msg::Node> & a, const std::vector<plansys2_msgs::msg::Node> & b)
{
  for (const auto & x : a) {
    for (const auto & y : b) {
      if (parser::pddl::checkNodeEquality(x, y)) {
        return true;
      }
    }
  }
  return false;
}
}  // namespace

bool
STNBTBuilder::interfere(
  const plansys2_msgs::msg::Tree & first, const plansys2_msgs::msg::Tree & second)
{
  std::vector<plansys2_msgs::msg::Node> adds_1, deletes_1, functions_1;
  std::vector<plansys2_msgs::msg::Node> adds_2, deletes_2, functions_2;
  written_effects(first, 0, false, adds_1, deletes_1, functions_1);
  written_effects(second, 0, false, adds_2, deletes_2, functions_2);
  return any_equal(adds_1, deletes_2) || any_equal(deletes_1, adds_2) ||
         any_equal(functions_1, functions_2);
}

plansys2_msgs::msg::Tree
STNBTBuilder::get_threat_effects(const ActionStamped & action) const
{
  // A non-durative action runs for a while too, and its effects can come at any moment of it
  if (action.type == ActionType::START && action.action.is_action()) {
    return action.action.get_at_end_effects();
  }
  return get_effects(action);
}

plansys2_msgs::msg::Tree
STNBTBuilder::get_effects(const ActionStamped & action) const
{
  if (action.type == ActionType::START) {
    return action.action.get_at_start_effects();
  }
  const bool at_end = action.type == ActionType::END || action.type == ActionType::INIT ||
    action.type == ActionType::TIMED;
  if (at_end) {
    return action.action.get_at_end_effects();
  }

  return plansys2_msgs::msg::Tree();
}

void
STNBTBuilder::prune_paths(Node::Ptr current, Node::Ptr previous) const
{
  std::set<const Node *> visited;
  prune_paths(current, previous, visited);
}

void
STNBTBuilder::prune_paths(
  Node::Ptr current, Node::Ptr previous, std::set<const Node *> & visited) const
{
  // Once per node: removing the arcs to current is done the first time
  if (!visited.insert(previous.get()).second) {
    return;
  }
  // Traverse the graph from the previous node to the root
  for (auto & in : previous->input_arcs) {
    prune_paths(current, std::get<0>(in), visited);
  }

  // Don't remove the link between the start and end node
  if (previous->action.type != ActionType::INIT &&
    current->action.type != ActionType::GOAL &&
    previous->action.time == current->action.time &&
    previous->action.expression == current->action.expression)
  {
    if (previous->action.type != ActionType::START) {
      std::cerr << "prune_paths: Expected previous action type is START. ";
      std::cerr << "Actual previous action type is ";
      std::cerr << to_string(previous->action.type) << std::endl;
    }
    if (current->action.type != ActionType::END) {
      std::cerr << "prune_paths: Expected current action type is END. ";
      std::cerr << "Actual current action type is ";
      std::cerr << to_string(current->action.type) << std::endl;
    }
    return;
  }

  auto it = previous->output_arcs.begin();
  while (it != previous->output_arcs.end()) {
    // Check for an output link to current
    if (std::get<0>(*it) == current) {
      // Find the corresponding input link
      auto in = std::find_if(
        current->input_arcs.begin(), current->input_arcs.end(),
        [&](std::tuple<Node::Ptr, double, double> arc) {
          return std::get<0>(arc) == previous;
        });
      // Remove the output and input links
      if (in != current->input_arcs.end()) {
        current->input_arcs.erase(in);
        it = previous->output_arcs.erase(it);
      } else {
        std::cerr << "prune_backards: Input arc could not be found" << std::endl;
      }
    } else {
      ++it;
    }
  }
}

bool
STNBTBuilder::check_paths(Node::Ptr current, Node::Ptr previous) const
{
  std::set<const Node *> visited;
  return check_paths(current, previous, visited);
}

bool
STNBTBuilder::check_paths(
  Node::Ptr current, Node::Ptr previous, std::set<const Node *> & visited) const
{
  // A node already seen did not reach previous
  if (!visited.insert(current.get()).second) {
    return false;
  }
  // Traverse the graph from the current node to the root
  for (auto & in : current->input_arcs) {
    if (check_paths(std::get<0>(in), previous, visited)) {
      return true;
    }
  }

  // Check if the current node is equal to the previous node
  if (current == previous) {
    return true;
  }

  return false;
}

Eigen::MatrixXd
STNBTBuilder::get_distance_matrix(const Graph::Ptr stn) const
{
  // Initialize the distance matrix as infinity.
  Eigen::MatrixXd dist = std::numeric_limits<double>::infinity() *
    Eigen::MatrixXd::Ones(stn->nodes.size(), stn->nodes.size());
  for (int i = 0; i < dist.rows(); i++) {
    dist(i, i) = 0.0;
  }

  // Extract the distances imposed by the STN.
  for (const auto & node : stn->nodes) {
    int row = node->node_num;
    for (const auto & arc : node->output_arcs) {
      auto child = std::get<0>(arc);
      int col = child->node_num;
      dist(row, col) = std::get<2>(arc);
      dist(col, row) = -std::get<1>(arc);
    }
  }

  // Solve the all-pairs shortest path problem.
  floyd_warshall(dist);

  return dist;
}

void
STNBTBuilder::floyd_warshall(Eigen::MatrixXd & dist) const
{
  for (Eigen::Index k = 0; k < dist.rows(); k++) {
    for (Eigen::Index i = 0; i < dist.rows(); i++) {
      for (Eigen::Index j = 0; j < dist.rows(); j++) {
        if (dist(i, k) == std::numeric_limits<double>::infinity() ||
          dist(k, j) == std::numeric_limits<double>::infinity())
        {
          continue;
        }
        if (dist(i, j) > (dist(i, k) + dist(k, j))) {
          dist(i, j) = dist(i, k) + dist(k, j);
        }
      }
    }
  }
}

void
STNBTBuilder::get_flow(
  const Node::Ptr node,
  const Node::Ptr prev_node,
  std::set<Node::Ptr> & used,
  const int & level,
  bool wrap,
  std::string & out) const
{
  int l = level;
  const auto action_id = to_action_id(node->action, action_time_precision_);

  // Waits for its parent: the node runs where it was used, or from its own start
  auto wait_for_parent = [&]() {
      std::string parent_id;
      std::string parent_type;
      if (prev_node) {
        parent_id = to_action_id(prev_node->action, action_time_precision_);
        parent_type = to_string(prev_node->action.type);
      }
      return t(l) + "<WaitAction action=\"" +
             action_id + " " + to_string(node->action.type) + " " +
             parent_id + " " + parent_type + "\"/>\n";
    };

  if (used.find(node) != used.end()) {
    out += wait_for_parent();
    return;
  }

  bool is_special = false;
  if (node->action.type == ActionType::END) {
    auto t_1 = to_int_time(node->action.time, action_time_precision_ + 1);
    auto t_2 = to_int_time(prev_node->action.time, action_time_precision_ + 1);
    if (prev_node->action.type != ActionType::START || (t_1 != t_2) ||
      (node->action.expression != prev_node->action.expression))
    {
      is_special = true;
    }
  }

  if (!is_special) {
    used.insert(node);
  }

  // An end reached from another parent than its start runs from its start: executed here too,
  // its action would run, and check its requirements, before it starts
  const bool leaf = node->output_arcs.empty() ||
    (node->output_arcs.size() == 1 &&
    std::get<0>(*node->output_arcs.begin())->action.type == ActionType::GOAL);
  if (is_special && leaf) {
    out += wait_for_parent();
    return;
  }

  if (node->output_arcs.size() == 0) {
    if (node->action.type == ActionType::END) {
      out += end_execution_block(node, l);
    } else if (node->action.type != ActionType::GOAL) {
      std::cerr << "get_flow: Unexpected action type" << std::endl;
    }
    return;
  } else if (node->output_arcs.size() == 1) {
    const auto child = std::get<0>(*node->output_arcs.begin());
    if (node->action.type == ActionType::END &&
      child->action.type == ActionType::GOAL)
    {
      out += end_execution_block(node, l);
      return;
    }
  }

  // Inside the Sequence of its parent when it is the only way on: Sequence(x, Sequence(y, z))
  // is Sequence(x, y, z), and the tree does not get one level deeper with each action
  const bool sequence = wrap && node->action.type != ActionType::INIT;
  if (sequence) {
    out += t(l) + "<Sequence name=\"" + action_id + "\">\n";
  } else {
    l = l - 1;  // Its content goes where its Sequence would be
  }

  if (node->action.type == ActionType::START) {
    out += start_execution_block(node, l + 1);
  } else if (node->action.type == ActionType::END) {
    auto t_1 = to_int_time(node->action.time, action_time_precision_ + 1);
    auto t_2 = to_int_time(prev_node->action.time, action_time_precision_ + 1);
    if (prev_node->action.type == ActionType::START && (t_1 == t_2) &&
      (node->action.expression == prev_node->action.expression))
    {
      out += end_execution_block(node, l + 1);
    } else {
      std::string parent_id;
      std::string parent_type;
      if (prev_node) {
        parent_id = to_action_id(prev_node->action, action_time_precision_);
        parent_type = to_string(prev_node->action.type);
      }

      out += t(l + 1) + "<WaitAction action=\"" +
        action_id + " " + to_string(node->action.type) + " " +
        parent_id + " " + parent_type + "\"/>\n";
    }
  }

  auto num_output_arcs = node->output_arcs.size();
  if (num_output_arcs > 1) {
    for (const auto & arc : node->output_arcs) {
      auto child = std::get<0>(arc);
      if (child->action.type == ActionType::GOAL) {
        num_output_arcs = num_output_arcs - 1;
        break;
      }
    }
  }

  int n = 0;
  if (num_output_arcs > 1) {
    out += t(l + 1) +
      "<Parallel success_count=\"" + std::to_string(num_output_arcs) +
      "\" failure_count=\"1\">\n";
    n = 1;
  }
  // Each branch of a Parallel, and the only child of the root, is a node of its own
  const bool wrap_children = num_output_arcs > 1 || node->action.type == ActionType::INIT;

  // Visit the end action first
  auto end_action_arc = node->output_arcs.end();
  if (node->action.type == ActionType::START) {
    end_action_arc = std::find_if(
      node->output_arcs.begin(), node->output_arcs.end(),
      std::bind(&STNBTBuilder::is_end, this, std::placeholders::_1, node->action));
    if (end_action_arc != node->output_arcs.end()) {
      const auto & next = std::get<0>(*end_action_arc);
      get_flow(next, node, used, l + n + 1, wrap_children, out);
    }
  }

  // Visit the rest of the output arcs
  for (const auto & child : node->output_arcs) {
    auto child_node = std::get<0>(child);
    if (end_action_arc != node->output_arcs.end()) {
      if (child_node == std::get<0>(*end_action_arc)) {
        continue;
      }
    }
    if (child_node->action.type != ActionType::GOAL) {
      const auto & next = std::get<0>(child);
      get_flow(next, node, used, l + n + 1, wrap_children, out);
    }
  }

  if (num_output_arcs > 1) {
    out += t(l + 1) + "</Parallel>\n";
  }

  if (sequence) {
    out += t(l) + "</Sequence>\n";
  }
}

std::string
STNBTBuilder::start_execution_block(
  const Node::Ptr node,
  const int & l) const
{
  std::string ret;
  std::string ret_aux = bt_start_action_;
  const std::string action_id = to_action_id(node->action, action_time_precision_);
  const std::string action_type = to_string(node->action.type);

  std::string wait_actions;
  for (const auto & prev : node->input_arcs) {
    const auto & prev_node = std::get<0>(prev);
    wait_actions = wait_actions + t(1) + "<WaitAction action=\"" +
      action_id + " " + action_type + " " +
      to_action_id(prev_node->action, action_time_precision_) + " " +
      to_string(prev_node->action.type) + "\"/>";

    if (prev != *node->input_arcs.rbegin()) {
      wait_actions = wait_actions + "\n";
    }
  }

  replace(ret_aux, "ACTION_ID", action_id);
  replace(ret_aux, "WAIT_PREV_ACTIONS", wait_actions);

  std::istringstream f(ret_aux);
  std::string line;
  while (std::getline(f, line)) {
    if (line != "") {
      ret = ret + t(l) + line + "\n";
    }
  }
  return ret;
}

std::string
STNBTBuilder::end_execution_block(
  const Node::Ptr node,
  const int & l) const
{
  std::string ret;
  std::string ret_aux = bt_end_action_;
  const std::string action_id = to_action_id(node->action, action_time_precision_);
  const std::string action_type = to_string(node->action.type);

  std::string check_actions;
  for (const auto & prev : node->input_arcs) {
    const auto & prev_node = std::get<0>(prev);
    check_actions = check_actions + t(1) + "<CheckAction action=\"" +
      action_id + " " + action_type + " " +
      to_action_id(prev_node->action, action_time_precision_) + " " +
      to_string(prev_node->action.type) + "\"/>";

    if (prev != *node->input_arcs.rbegin()) {
      check_actions = check_actions + "\n";
    }
  }

  replace(ret_aux, "ACTION_ID", action_id);
  replace(ret_aux, "CHECK_PREV_ACTIONS", check_actions);

  std::istringstream f(ret_aux);
  std::string line;
  while (std::getline(f, line)) {
    if (line != "") {
      ret = ret + t(l) + line + "\n";
    }
  }
  return ret;
}

void
STNBTBuilder::get_flow_dotgraph(
  Node::Ptr node,
  std::set<std::string> & edges)
{
  std::set<const Node *> visited;
  get_flow_dotgraph(node, edges, visited);
}

void
STNBTBuilder::get_flow_dotgraph(
  Node::Ptr node, std::set<std::string> & edges, std::set<const Node *> & visited)
{
  // Once per node: it runs on every feedback, from every node
  if (!visited.insert(node.get()).second) {
    return;
  }
  for (const auto & arc : node->output_arcs) {
    auto child = std::get<0>(arc);
    std::string edge = std::to_string(node->node_num) + "->" + std::to_string(child->node_num) +
      ";\n";
    edges.insert(edge);
    get_flow_dotgraph(child, edges, visited);
  }
}

std::string
STNBTBuilder::get_node_dotgraph(
  Node::Ptr node,
  std::shared_ptr<std::map<std::string, ActionExecutionInfo>> action_map)
{
  std::stringstream ss;
  ss << t(2) << node->node_num << " [label=\"";
  ss << node->action.action.get_action_string();
  ss << " " << to_string(node->action.type) << "\"";
  ss << "labeljust=c,style=filled";

  auto status = get_action_status(node->action, action_map);
  switch (status) {
    case ActionExecutor::RUNNING:
      ss << ",color=blue,fillcolor=skyblue";
      break;
    case ActionExecutor::SUCCESS:
      ss << ",color=green4,fillcolor=seagreen2";
      break;
    case ActionExecutor::FAILURE:
      ss << ",color=red,fillcolor=pink";
      break;
    case ActionExecutor::CANCELLED:
      ss << ",color=red,fillcolor=pink";
      break;
    case ActionExecutor::IDLE:
    case ActionExecutor::DEALING:
    default:
      ss << ",color=yellow3,fillcolor=lightgoldenrod1";
      break;
  }
  ss << "];\n";
  return ss.str();
}

ActionExecutor::Status
STNBTBuilder::get_action_status(
  ActionStamped action_stamped,
  std::shared_ptr<std::map<std::string, ActionExecutionInfo>> action_map)
{
  if (!action_map) {
    return ActionExecutor::IDLE;  // Nothing is executing yet
  }
  auto index = "(" + action_stamped.action.get_action_string() + "):" +
    std::to_string(to_int_time(action_stamped.time, action_time_precision_));
  if (action_map->find(index) != action_map->end()) {
    if ((*action_map)[index].action_executor) {
      return (*action_map)[index].action_executor->get_internal_status();
    } else {
      return ActionExecutor::IDLE;
    }
  } else {
    return ActionExecutor::IDLE;
  }
}

std::string
STNBTBuilder::add_dot_graph_legend(
  int level_counter,
  int node_counter)
{
  std::stringstream ss;
  int legend_counter = level_counter;
  int legend_node_counter = node_counter;
  ss << t(1);
  ss << "subgraph cluster_" << legend_counter++ << " {\n";
  ss << t(2);
  ss << "label = \"Legend\";\n";

  ss << t(2);
  ss << "subgraph cluster_" << legend_counter++ << " {\n";
  ss << t(3);
  ss << "label = \"Plan Action Start (sec): X.X s\n Duration (sec): X.X s\";\n";
  ss << t(3);
  ss << "style = rounded;\n";
  ss << t(3);
  ss << "color = yellow3;\n";
  ss << t(3);
  ss << "bgcolor = lemonchiffon;\n";
  ss << t(3);
  ss << "labeljust = l;\n";
  ss << t(3);
  ss << legend_node_counter++ <<
    " [label=\n\"Finished action\n\",labeljust=c,style=filled,color=green4,fillcolor=seagreen2];\n";
  ss << t(3);
  ss << legend_node_counter++ <<
    " [label=\n\"Failed action\n\",labeljust=c,style=filled,color=red,fillcolor=pink];\n";
  ss << t(3);
  ss << legend_node_counter++ <<
    " [label=\n\"Current action\n\",labeljust=c,style=filled,color=blue,fillcolor=skyblue];\n";
  ss << t(3);
  ss << legend_node_counter++ << " [label=\n\"Future action\n\",labeljust=c,style=filled," <<
    "color=yellow3,fillcolor=lightgoldenrod1];\n";
  ss << t(2);
  ss << "}\n";

  ss << t(2);
  for (int i = node_counter; i < legend_node_counter; i++) {
    if (i > node_counter) {
      ss << "->";
    }
    ss << i;
  }
  ss << " [style=invis];\n";

  ss << t(1);
  ss << "}\n";

  return ss.str();
}

void
STNBTBuilder::print_graph(const plansys2::Graph::Ptr graph) const
{
  print_node(graph->nodes.front(), 0);
}

void
STNBTBuilder::print_node(const plansys2::Node::Ptr node, int level) const
{
  std::cerr << t(level) << "(" << node->node_num << ") ";
  if (node->action.type == ActionType::START) {
    std::cerr << node->action.time;
  } else {
    std::cerr << node->action.time + node->action.duration;
  }
  std::cerr << ": (" << node->action.action.get_action_name();
  for (const auto & param : node->action.action.get_action_params()) {
    std::cerr << " " << param.name;
  }
  std::cerr << ")_" << to_string(node->action.type);
  std::cerr << "  [" << node->action.duration << "]";
  for (const auto & arc : node->output_arcs) {
    auto lower = std::get<1>(arc);
    auto upper = std::get<2>(arc);
    std::cerr << " [" << lower << ", " << upper << "]";
  }
  std::cerr << std::endl;

  for (const auto & arc : node->output_arcs) {
    auto child = std::get<0>(arc);
    print_node(child, level + 1);
  }
}

void
STNBTBuilder::print_arcs(const plansys2::Graph::Ptr graph) const
{
  for (const auto & node : graph->nodes) {
    int row = node->node_num;
    for (const auto & arc : node->output_arcs) {
      auto child = std::get<0>(arc);
      int col = child->node_num;

      std::string error_msg = std::to_string(node->node_num) +
        " -> " +
        std::to_string(child->node_num) +
        " : " +
        to_action_id(node->action, action_time_precision_) + "_" + to_string(node->action.type) +
        " -> " +
        to_action_id(child->action, action_time_precision_) + "_" + to_string(child->action.type) +
        " : " +
        "upper = dist(" + std::to_string(row) + ", " + std::to_string(col) + ") = " +
        std::to_string(std::get<2>(arc)) +
        ", " +
        "lower = dist(" + std::to_string(col) + ", " + std::to_string(row) + ") = " +
        std::to_string(-1.0 * std::get<1>(arc));

      std::cerr << error_msg << std::endl;
    }
  }
}

bool
STNBTBuilder::is_end(
  const std::tuple<Node::Ptr, double, double> & edge,
  const ActionStamped & action) const
{
  const auto & node = std::get<0>(edge);
  auto t_1 = to_int_time(node->action.time, action_time_precision_ + 1);
  auto t_2 = to_int_time(action.time, action_time_precision_ + 1);
  return action.type == ActionType::START &&
         node->action.type == ActionType::END &&
         (t_1 == t_2) && (node->action.expression == action.expression);
}

std::string
STNBTBuilder::t(const int & level) const
{
  std::string ret;
  for (int i = 0; i < level; i++) {
    ret = ret + "  ";
  }
  return ret;
}

}  // namespace plansys2

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(plansys2::STNBTBuilder, plansys2::BTBuilder)
