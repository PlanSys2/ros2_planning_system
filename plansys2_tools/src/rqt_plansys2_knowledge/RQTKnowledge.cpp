// Copyright 2021 Intelligent Robotics Lab
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

#include <unistd.h>

#include <QDebug>
#include <QTime>
#include <QTimer>
#include <QPushButton>
#include <QFileDialog>
#include <QMessageBox>
#include <QPainter>
#include <QCheckBox>
#include <algorithm>
#include <memory>
#include <utility>
#include <string>

#include <pluginlib/class_list_macros.hpp>

#include "rqt_plansys2_knowledge/RQTKnowledge.hpp"
#include "rqt_plansys2_knowledge/KnowledgeTree.hpp"


#include "plansys2_problem_expert/ProblemExpertClient.hpp"

namespace rqt_plansys2_knowledge
{

using std::placeholders::_1;

RQTKnowledge::RQTKnowledge()
: rqt_gui_cpp::Plugin(),
  widget_(0)
{
  setObjectName("RQTKnowledge");
}

void RQTKnowledge::initPlugin(qt_gui_cpp::PluginContext & context)
{
  widget_ = new QWidget();
  ui_.setupUi(widget_);

  if (context.serialNumber() > 1) {
    widget_->setWindowTitle(
      widget_->windowTitle() + " (" +
      QString::number(context.serialNumber()) + ")");
  }
  context.addWidget(widget_);

  knowledge_tree_ = new KnowledgeTree();
  ui_.gridLayout->addWidget(knowledge_tree_);

  knowledge_tree_->setColumnCount(3);
  knowledge_tree_->setHeaderLabels({"Type", "Item", "Type / Value"});

  controller_spin_timer_ = new QTimer(this);
  connect(controller_spin_timer_, SIGNAL(timeout()), this, SLOT(spin_loop()));
  controller_spin_timer_->start(10);
  need_repaint_ = false;

  knowledge_sub_ = node_->create_subscription<plansys2_msgs::msg::Knowledge>(
    "problem_expert/knowledge", rclcpp::QoS(100).transient_local(),
    std::bind(&RQTKnowledge::knowledge_callback, this, _1));

  problem_ = std::make_shared<plansys2::ProblemExpertClient>();
}

void RQTKnowledge::shutdownPlugin()
{
}

void
RQTKnowledge::knowledge_callback(plansys2_msgs::msg::Knowledge::UniquePtr msg)
{
  last_msg_ = std::move(msg);
  need_repaint_ = true;
}

void
RQTKnowledge::saveSettings(
  qt_gui_cpp::Settings & plugin_settings, qt_gui_cpp::Settings & instance_settings) const
{
  (void)plugin_settings;
  (void)instance_settings;
}

void
RQTKnowledge::spin_loop()
{
  if (need_repaint_) {
    knowledge_tree_->clearAllItems();

    // Instances
    for (const std::string & instance : last_msg_->instances) {
      auto instance_item = new QTreeWidgetItem();
      instance_item->setText(0, QString("instance"));
      instance_item->setText(1, QString(instance.c_str()));

      auto complete_instance = problem_->getInstance(instance);
      if (complete_instance.has_value()) {
        instance_item->setText(2, QString(complete_instance.value().type.c_str()));
      }

      instance_item->setBackground(0, Qt::lightGray);
      instance_item->setBackground(1, Qt::lightGray);
      instance_item->setBackground(2, Qt::lightGray);
      knowledge_tree_->addTopLevelItem(instance_item);
    }

    // Predicates
    for (const std::string & predicate : last_msg_->predicates) {
      auto predicate_item = new QTreeWidgetItem();
      predicate_item->setText(0, QString("predicate"));
      predicate_item->setText(1, QString(predicate.c_str()));
      predicate_item->setBackground(0, Qt::yellow);
      predicate_item->setBackground(1, Qt::yellow);
      predicate_item->setBackground(2, Qt::yellow);
      knowledge_tree_->addTopLevelItem(predicate_item);
    }

    // Functions (Value?)
    for (const std::string & function : last_msg_->functions) {
      auto function_item = new QTreeWidgetItem();
      function_item->setText(0, QString("function"));
      function_item->setText(1, QString(function.c_str()));

      auto complete_function = problem_->getFunction(function);
      if (complete_function.has_value()) {
        function_item->setText(2, QString::number(complete_function.value().value));
      }

      function_item->setBackground(0, Qt::blue);
      function_item->setBackground(1, Qt::blue);
      function_item->setBackground(2, Qt::blue);
      knowledge_tree_->addTopLevelItem(function_item);
    }

    // Goal
    auto goal_item = new QTreeWidgetItem();
    goal_item->setText(0, QString("goal"));
    if (last_msg_->goal == "") {
      goal_item->setText(1, QString("No goal"));
      goal_item->setBackground(0, Qt::red);
      goal_item->setBackground(1, Qt::red);
      goal_item->setBackground(2, Qt::red);
    } else {
      goal_item->setText(1, QString(last_msg_->goal.c_str()));
      goal_item->setBackground(0, Qt::green);
      goal_item->setBackground(1, Qt::green);
      goal_item->setBackground(2, Qt::green);
    }
    knowledge_tree_->addTopLevelItem(goal_item);

    knowledge_tree_->parentWidget()->repaint();
    need_repaint_ = false;
  }
}

void
RQTKnowledge::restoreSettings(
  const qt_gui_cpp::Settings & plugin_settings, const qt_gui_cpp::Settings & instance_settings)
{
  (void)plugin_settings;
  (void)instance_settings;
}


}  // namespace rqt_plansys2_knowledge

PLUGINLIB_EXPORT_CLASS(rqt_plansys2_knowledge::RQTKnowledge, rqt_gui_cpp::Plugin)
