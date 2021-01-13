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

#include <string>
#include <memory>
#include <vector>
#include <set>
#include <algorithm>

#include "plansys2_executor/BTBuilder.hpp"

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_domain_expert/Types.hpp"
#include "plansys2_executor/Utils.hpp"

#include "rclcpp/rclcpp.hpp"

namespace plansys2
{

BTBuilder::BTBuilder(
  rclcpp::Node::SharedPtr node)
{
  domain_client_ = std::make_shared<plansys2::DomainExpertClient>(node);
  problem_client_ = std::make_shared<plansys2::ProblemExpertClient>(node);
}

std::string
BTBuilder::get_tree(const Plan & current_plan)
{
  auto levels = get_plan_actions(current_plan);

  // Test the required action for each action
  for (int i = 1; i < levels.size(); i++) {
    int level_comp = i - 1;
    while (level_comp >= 0 && !level_satisfied(levels[i])) {
      check_connections(levels[level_comp], levels[i]);
      level_comp--;
    }
  }

  // Test if requirement is satisfied by pre-existing knowledge
  for (auto & level : levels) {
    for (auto & action_unit : level->action_units) {
      for (auto & req : action_unit->at_start_reqs) {
        if (!req->satisfied) {
          req->satisfied = problem_client_->existPredicate(Predicate(req->requirement));
        }
      }
      for (auto & req : action_unit->over_all_reqs) {
        if (!req->satisfied) {
          req->satisfied = problem_client_->existPredicate(Predicate(req->requirement));
        }
      }
      for (auto & req : action_unit->at_end_reqs) {
        if (!req->satisfied) {
          req->satisfied = problem_client_->existPredicate(Predicate(req->requirement));
        }
      }
    }
  }

  // Check how many independent flows there are
  int root_counters = 0;
  for (auto & level : levels) {
    for (auto & action_unit : level->action_units) {
      if (in_cardinality(action_unit) == 0) {
        root_counters++;
      }
    }
  }

  print_levels(levels);

  std::cerr << "==============> Roots = " << root_counters << std::endl;

  std::string bt_plan;

  if (root_counters > 1) {
    bt_plan = std::string("<root main_tree_to_execute=\"MainTree\">\n") +
      t(1) + "<BehaviorTree ID=\"MainTree\">\n" +
      t(2) + "<Parallel success_threshold=\"" + std::to_string(root_counters) +
      "\" failure_threshold=\"1\">\n";

    for (auto & level : levels) {
      for (auto & action_unit : level->action_units) {
        if (in_cardinality(action_unit) == 0) {
          std::set<ActionUnit::Ptr> used_actions;
          bt_plan = bt_plan + get_flow_tree(action_unit, used_actions, 3);
        }
      }
    }

    bt_plan = bt_plan + t(2) + "</Parallel>\n" +
      t(1) + "</BehaviorTree>\n</root>\n";
  } else {
    bt_plan = std::string("<root main_tree_to_execute=\"MainTree\">\n") +
      t(1) + "<BehaviorTree ID=\"MainTree\">\n";

    for (auto & level : levels) {
      for (auto & action_unit : level->action_units) {
        if (in_cardinality(action_unit) == 0) {
          std::set<ActionUnit::Ptr> used_actions;
          bt_plan = bt_plan + get_flow_tree(action_unit, used_actions, 2);
        }
      }
    }

    bt_plan = bt_plan + t(1) + "</BehaviorTree>\n</root>\n";
  }

  return bt_plan;

  return "";
}

std::set<ActionUnit::Ptr>
BTBuilder::pred(ActionUnit::Ptr action_unit)
{
  std::set<ActionUnit::Ptr> deps;
  for (auto & req : action_unit->at_start_reqs) {
    for (auto & effect_con : req->effect_connections) {
      deps.insert(effect_con->action);
    }
  }
  for (auto & req : action_unit->over_all_reqs) {
    for (auto & effect_con : req->effect_connections) {
      deps.insert(effect_con->action);
    }
  }
  for (auto & req : action_unit->at_end_reqs) {
    for (auto & effect_con : req->effect_connections) {
      deps.insert(effect_con->action);
    }
  }

  return deps;
}

std::set<ActionUnit::Ptr>
BTBuilder::succ(ActionUnit::Ptr action_unit)
{
  std::set<ActionUnit::Ptr> deps;
  for (auto & effect : action_unit->at_start_effects) {
    for (auto & req_con : effect->requirement_connections) {
      deps.insert(req_con->action);
    }
  }
  for (auto & effect : action_unit->at_end_effects) {
    for (auto & req_con : effect->requirement_connections) {
      deps.insert(req_con->action);
    }
  }

  return deps;
}

int
BTBuilder::in_cardinality(ActionUnit::Ptr action_unit)
{
  return pred(action_unit).size();
}

int
BTBuilder::out_cardinality(ActionUnit::Ptr action_unit)
{
  return succ(action_unit).size();
}

std::string
BTBuilder::t(int level)
{
  std::string ret;
  for (int i = 0; i < level; i++) {
    ret = ret + "  ";
  }
  return ret;
}

std::string
BTBuilder::get_flow_tree(
  ActionUnit::Ptr root_flow,
  std::set<ActionUnit::Ptr> & used_actions, int level)
{
  std::string ret;
  int l = level;

  used_actions.insert(root_flow);

  if (out_cardinality(root_flow) == 0) {
    if (in_cardinality(root_flow) > 1) {
      ret = t(l) + "<Sequence name=\"" + root_flow->action + ":" +
        std::to_string(root_flow->time) + "\">\n";

      for (auto & action : pred(root_flow)) {
        ret = ret + t(l + 1) + "<WaitAction action=\"" + action->action + ":" +
          std::to_string(action->time) + "\"/>\n";
      }

      ret = ret + execution_block(root_flow->action, root_flow->time, l + 1) +
        t(l) + "</Sequence>\n";
    } else {
      ret = execution_block(root_flow->action, root_flow->time, l);
    }
  }

  if (out_cardinality(root_flow) == 1) {
    ret = t(l) + "<Sequence name=\"" + root_flow->action + ":" +
      std::to_string(root_flow->time) + "\">\n";

    if (in_cardinality(root_flow) > 1) {
      for (auto & action : pred(root_flow)) {
        ret = ret + t(l + 1) + "<WaitAction action=\"" + action->action + ":" +
          std::to_string(action->time) + "\"/>\n";
      }
    }

    ret = ret + execution_block(root_flow->action, root_flow->time, l + 1) +
      get_flow_tree(*succ(root_flow).begin(), used_actions, l + 1) +
      t(l) + "</Sequence>\n";
  }

  if (out_cardinality(root_flow) > 1) {
    ret = t(l) + "<Sequence name=\"" + root_flow->action + ":" +
      std::to_string(root_flow->time) + "\">\n";

    if (in_cardinality(root_flow) > 1) {
      for (auto & action : pred(root_flow)) {
        ret = ret + t(l + 1) + "<WaitAction action=\"" + action->action + ":" +
          std::to_string(action->time) + "\"/>\n";
      }
    }

    ret = ret +
      execution_block(root_flow->action, root_flow->time, l + 1) +
      t(l + 1) + "<Parallel success_threshold=\"" + std::to_string(succ(root_flow).size()) +
      "\"  failure_threshold=\"1\">\n";

    for (auto & action : succ(root_flow)) {
      ret = ret + get_flow_tree(action, used_actions, l + 2);
    }
    ret = ret + t(l + 1) + "</Parallel>\n";
    ret = ret + t(l) + "</Sequence>\n";
  }

  return ret;
}

std::string
BTBuilder::execution_block(const std::string & action, int plan_time, int l)
{
  std::string ret;

  ret = ret + t(l) + "<Sequence name=\"" + action + ":" +
    std::to_string(plan_time) + "\">\n";
  ret = ret + t(l + 1) + "<WaitAtStartReq action=\"" + action + ":" +
    std::to_string(plan_time) + "\"/>\n";
  ret = ret + t(l + 1) + "<ApplyAtStartEffect action=\"" + action + ":" +
    std::to_string(plan_time) + "\"/>\n";
  ret = ret + t(l + 1) + "<Parallel success_threshold=\"2\" failure_threshold=\"1\">\n";
  ret = ret + t(l + 2) + "<CheckOverAllReq action=\"" + action + ":" +
    std::to_string(plan_time) + "\"/>\n";
  ret = ret + t(l + 2) + "<ExecuteAction action=\"" + action + ":" +
    std::to_string(plan_time) + "\"/>\n";
  ret = ret + t(l + 1) + "</Parallel>\n";
  ret = ret + t(l + 1) + "<CheckAtEndReq action=\"" + action + ":" +
    std::to_string(plan_time) + "\"/>\n";
  ret = ret + t(l + 1) + "<ApplyAtEndEffect action=\"" + action + ":" +
    std::to_string(plan_time) + "\"/>\n";
  ret = ret + t(l) + "</Sequence>\n";

  return ret;
}

void
BTBuilder::check_connections(ExecutionLevel::Ptr up_level, ExecutionLevel::Ptr down_level)
{
  for (auto & down_action_unit : down_level->action_units) {
    for (auto & req : down_action_unit->at_start_reqs) {
      if (!req->satisfied) {
        for (auto & up_action_unit : up_level->action_units) {
          
          for (auto & effect : up_action_unit->at_start_effects) {
            
            std::cerr << effect->effect << " <-1-> " << req->requirement << std::endl;
            if (req->requirement == effect->effect) {

              std::cerr << "\tat_start_effects [" << effect->action->action << "] satisfy at_start_req [" << req->requirement << "]" << std::endl;

              req->satisfied = true;
              req->effect_connections.push_back(effect);
              effect->requirement_connections.push_back(req);
            }
          }
          for (auto & effect : up_action_unit->at_end_effects) {
            std::cerr << effect->effect << " <-2-> " << req->requirement << std::endl;
            if (req->requirement == effect->effect) {

              std::cerr << "\tat_end_effects [" << effect->action->action << "] satisfy at_start_req [" << req->requirement << "]" << std::endl;

              req->satisfied = true;
              req->effect_connections.push_back(effect);
              effect->requirement_connections.push_back(req);
            }
          }
        }
      }
    }
    for (auto & req : down_action_unit->over_all_reqs) {
      if (!req->satisfied) {
        for (auto & up_action_unit : up_level->action_units) {
          
          for (auto & effect : up_action_unit->at_start_effects) {
            std::cerr << effect->effect << " <-3-> " << req->requirement << std::endl;
            if (req->requirement == effect->effect) {
 
               std::cerr << "\tat_start_effects [" << effect->action->action << "] satisfy over_all_req [" << req->requirement << "]" << std::endl;

              req->satisfied = true;
              req->effect_connections.push_back(effect);
              effect->requirement_connections.push_back(req);
            }
          }
          for (auto & effect : up_action_unit->at_end_effects) {
            std::cerr << effect->effect << " <-4-> " << req->requirement << std::endl;

            if (req->requirement == effect->effect) {

               std::cerr << "\tat_end_effects [" << effect->action->action << "] satisfy over_all_req [" << req->requirement << "]" << std::endl;

              req->satisfied = true;
              req->effect_connections.push_back(effect);
              effect->requirement_connections.push_back(req);
            }
          }
        }
      }
    }

    for (auto & req : down_action_unit->at_end_reqs) {
      if (!req->satisfied) {
        for (auto & up_action_unit : up_level->action_units) {
          
          for (auto & effect : up_action_unit->at_start_effects) {

            std::cerr << effect->effect << " <-4-> " << req->requirement << std::endl;

            if (req->requirement == effect->effect) {

               std::cerr << "\tat_start_effects [" << effect->action->action << "] satisfy at_end_req [" << req->requirement << "]" << std::endl;

              req->satisfied = true;
              req->effect_connections.push_back(effect);
              effect->requirement_connections.push_back(req);
            }
          }
          for (auto & effect : up_action_unit->at_end_effects) {

            std::cerr << effect->effect << " <-5-> " << req->requirement << std::endl;

            if (req->requirement == effect->effect) {

               std::cerr << "\tat_end_effects [" << effect->action->action << "] satisfy at_end_req [" << req->requirement << "]" << std::endl;

              req->satisfied = true;
              req->effect_connections.push_back(effect);
              effect->requirement_connections.push_back(req);
            }
          }
        }
      }
    }



  }
}


bool
BTBuilder::level_satisfied(ExecutionLevel::Ptr level)
{
  bool ret = true;
  for (auto & action_unit : level->action_units) {
    for (auto & req : action_unit->at_start_reqs) {
      ret = ret && req->satisfied;
    }
    for (auto & req : action_unit->over_all_reqs) {
      ret = ret && req->satisfied;
    }
    for (auto & req : action_unit->at_end_reqs) {
      ret = ret && req->satisfied;
    }
  }

  return ret;
}

void
BTBuilder::print_levels(std::vector<ExecutionLevel::Ptr> & levels)
{
  int counter_level = 0;
  for (auto & level : levels) {
    std::cout << "====== Level " << counter_level++ << " [" << level->time << "]" << std::endl;

    for (const auto & action_unit : level->action_units) {
      std::cout << "\t" << action_unit->action << "\tin_cardinality: " <<
        in_cardinality(action_unit) << "\tout_cardinality: " <<
        out_cardinality(action_unit) << std::endl;

      std::cout << "\t\tReqs: " << std::endl;

      // At Start Reqs
      for (const auto & req : action_unit->at_start_reqs) {
        std::cout << "\t\t\tReq At Start: " << req->requirement <<
        (req->satisfied ? " Satisfied" : " Not satisfied") << std::endl;
        
        for (auto & action : req->effect_connections) {
          std::cout << "\t\t\t\t" << action->action->action << std::endl;
        }
      }
      for (const auto & req : action_unit->at_start_neg_reqs) {
        std::cout << "\t\t\tReq At Start: NOT " << req->requirement <<
        (req->satisfied ? " Satisfied" : " Not satisfied") << std::endl;

        for (auto & action : req->effect_connections) {
          std::cout << "\t\t\t\t" << action->action->action << std::endl;
        }
      }

      // Over All Reqs
      for (const auto & req : action_unit->over_all_reqs) {
        std::cout << "\t\t\tReq Over All: " << req->requirement <<
        (req->satisfied ? " Satisfied" : " Not satisfied") << std::endl;

        for (auto & action : req->effect_connections) {
          std::cout << "\t\t\t\t" << action->action->action << std::endl;
        }
      }

      for (const auto & req : action_unit->at_start_neg_reqs) {
        std::cout << "\t\t\tReq Over all: NOT " << req->requirement <<
        (req->satisfied ? " Satisfied" : "  Not satisfied") << std::endl;

        for (auto & action : req->effect_connections) {
          std::cout << "\t\t\t\t" << action->action->action << std::endl;
        }
      }

      // At End Reqs
      for (const auto & req : action_unit->at_end_reqs) {
        std::cout << "\t\t\tReq At End: " << req->requirement <<
        (req->satisfied ? " Satisfied" : " Not satisfied") << std::endl;

        for (auto & action : req->effect_connections) {
          std::cout << "\t\t\t\t" << action->action->action << std::endl;
        }
      }

      for (const auto & req : action_unit->at_start_neg_reqs) {
        std::cout << "\t\t\tReq At End: NOT " << req->requirement <<
        (req->satisfied ? " Satisfied" : " Not satisfied") << std::endl;

        for (auto & action : req->effect_connections) {
          std::cout << "\t\t\t\t" << action->action->action << std::endl;
        }
      }

      std::cout << "\t\tEffects: " << std::endl;

      // At Start Effect
      for (const auto & effect : action_unit->at_start_effects) {
        std::cout << "\t\t\tEffect At Start: " << effect->effect << std::endl;

        for (auto & req : effect->requirement_connections) {
          std::cout << "\t\t\t\t" << req->action->action << std::endl;
        }
      }
      for (const auto & effect : action_unit->at_start_neg_effects) {
        std::cout << "\t\t\tEffect At Start: NOT " << effect->effect << std::endl;

        for (auto & req : effect->requirement_connections) {
          std::cout << "\t\t\t\t" << req->action->action << std::endl;
        }
      }
      // At End Effect
      for (const auto & effect : action_unit->at_end_effects) {
        std::cout << "\t\t\tEffect At End: " << effect->effect << std::endl;

        for (auto & req : effect->requirement_connections) {
          std::cout << "\t\t\t\t" << req->action->action << std::endl;
        }
      }
      for (const auto & effect : action_unit->at_end_neg_effects) {
        std::cout << "\t\t\tEffect At Start: NOT " << effect->effect << std::endl;

        for (auto & req : effect->requirement_connections) {
          std::cout << "\t\t\t\t" << req->action->action << std::endl;
        }
      }
    }
  }
}

std::vector<ExecutionLevel::Ptr>
BTBuilder::get_plan_actions(const Plan & plan)
{
  std::vector<ExecutionLevel::Ptr> ret;

  auto current_level = ExecutionLevel::make_shared();
  ret.push_back(current_level);

  int last_time = 0;
  for (auto & item : plan) {
    int time = static_cast<int>(item.time * 1000);
    if (time > last_time) {
      last_time = time;
      current_level = ExecutionLevel::make_shared();
      ret.push_back(current_level);

      current_level->time = time;
    }

    auto action_unit = ActionUnit::make_shared();
    current_level->action_units.push_back(action_unit);

    action_unit->action = item.action;
    action_unit->time = current_level->time;

    auto dur_action = get_action_from_string(item.action, domain_client_);
    std::vector<plansys2::Predicate> at_start_requirements;
    dur_action->at_start_requirements.getPredicates(at_start_requirements);

    std::vector<plansys2::Predicate> over_all_requirements;
    dur_action->over_all_requirements.getPredicates(over_all_requirements);
    std::vector<plansys2::Predicate> at_end_requirements;
    dur_action->at_end_requirements.getPredicates(at_end_requirements);

    std::vector<plansys2::Predicate> requirements;

    for (const auto & r : at_start_requirements) {
      auto req = RequirementConnection::make_shared();

      if (r.negative) {
        action_unit->at_start_neg_reqs.push_back(req);
      } else {
        action_unit->at_start_reqs.push_back(req);
      }

      req->requirement = r.toString();
      req->action = action_unit;
    }

    for (const auto & r : at_end_requirements) {
      auto req = RequirementConnection::make_shared();

      if (r.negative) {
        action_unit->at_end_neg_reqs.push_back(req);
      } else {
        action_unit->at_end_reqs.push_back(req);
      }

      req->requirement = r.toString();
      req->action = action_unit;
    }

    for (const auto & r : over_all_requirements) {
      auto req = RequirementConnection::make_shared();

      if (r.negative) {
        action_unit->over_all_neg_reqs.push_back(req);
      } else {
        action_unit->over_all_reqs.push_back(req);
      }

      req->requirement = r.toString();
      req->action = action_unit;
    }

    std::vector<plansys2::Predicate> at_start_effects;
    dur_action->at_start_effects.getPredicates(at_start_effects);
    std::vector<plansys2::Predicate> at_end_effects;
    dur_action->at_end_effects.getPredicates(at_end_effects);

    for (const auto & e : at_start_effects) {
      auto effect = EffectConnection::make_shared();
      
      if (e.negative) {
        action_unit->at_start_neg_effects.push_back(effect);
      } else {
        action_unit->at_start_effects.push_back(effect);
      }

      effect->effect = e.toString();
      effect->action = action_unit;
    }

    for (const auto & e : at_end_effects) {
      auto effect = EffectConnection::make_shared();
      
      if (e.negative) {
        action_unit->at_end_neg_effects.push_back(effect);
      } else {
        action_unit->at_end_effects.push_back(effect);
      }

      effect->effect = e.toString();
      effect->action = action_unit;
    }
  }

  return ret;
}

bool operator<(const ActionUnit::Ptr & op1, const ActionUnit::Ptr & op2)
{
  std::string op1_str = op1->action + ":" + std::to_string(op1->time);
  std::string op2_str = op2->action + ":" + std::to_string(op2->time);

  return op1_str < op2_str;
}

}  // namespace plansys2
