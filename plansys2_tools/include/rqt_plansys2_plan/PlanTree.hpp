// Copyright 2022 Intelligent Robotics Lab
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

#ifndef RQT_PLANSYS2_PLAN__PLANTREE_HPP_
#define RQT_PLANSYS2_PLAN__PLANTREE_HPP_

#include <QWidget>
#include <QTreeWidget>

namespace rqt_plansys2_plan
{

class PlanTree : public QTreeWidget
{
public:
  PlanTree();

  void clearAllItems();
};

}  // namespace rqt_plansys2_plan

#endif  // RQT_PLANSYS2_PLAN__PLANTREE_HPP_
