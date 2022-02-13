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


#include <QWidget>
#include <QTreeWidget>

#include "rqt_plansys2_plan/PlanTree.hpp"

namespace rqt_plansys2_plan
{

PlanTree::PlanTree()
: QTreeWidget()
{
}

void
PlanTree::clearAllItems()
{
  int rows = topLevelItemCount();

  while (topLevelItemCount() > 0) {
    rowsAboutToBeRemoved(rootIndex(), 0, 0);
    delete takeTopLevelItem(0);
    rowsRemoved(rootIndex(), 0, 0);/* code */
  }
}

}  // namespace rqt_plansys2_plan
