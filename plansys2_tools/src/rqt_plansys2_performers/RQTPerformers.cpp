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
#include "rqt_plansys2_performers/RQTPerformers.hpp"
#include "rqt_plansys2_performers/PerformersTree.hpp"


#include "plansys2_problem_expert/ProblemExpertClient.hpp"

namespace rqt_plansys2_performers
{

using std::placeholders::_1;

RQTPerformers::RQTPerformers()
: rqt_gui_cpp::Plugin(),
  widget_(0)
{
  setObjectName("RQTPerformers");
}

void RQTPerformers::initPlugin(qt_gui_cpp::PluginContext & context)
{
  widget_ = new QWidget();
  ui_.setupUi(widget_);

  if (context.serialNumber() > 1) {
    widget_->setWindowTitle(
      widget_->windowTitle() + " (" +
      QString::number(context.serialNumber()) + ")");
  }
  context.addWidget(widget_);

  performers_tree_ = new PerformersTree();
  ui_.gridLayout->addWidget(performers_tree_);

  performers_tree_->setColumnCount(5);
  performers_tree_->setHeaderLabels(
    {"Performer", "Action", "Status", "Status Recency",
      "Specialized Arguments"});

  controller_spin_timer_ = new QTimer(this);
  connect(controller_spin_timer_, SIGNAL(timeout()), this, SLOT(spin_loop()));
  controller_spin_timer_->start(100);
  need_update_ = false;

  performers_sub_ = node_->create_subscription<plansys2_msgs::msg::ActionPerformerStatus>(
    "performers_status", rclcpp::QoS(100).reliable(),
    std::bind(&RQTPerformers::performers_callback, this, _1));

  problem_ = std::make_shared<plansys2::ProblemExpertClient>();
}

void RQTPerformers::shutdownPlugin()
{
}

void
RQTPerformers::performers_callback(plansys2_msgs::msg::ActionPerformerStatus::UniquePtr msg)
{
  performers_info_[msg->node_name] = std::move(msg);
  need_update_ = true;
}

void
RQTPerformers::saveSettings(
  qt_gui_cpp::Settings & plugin_settings, qt_gui_cpp::Settings & instance_settings) const
{
  (void)plugin_settings;
  (void)instance_settings;
}

std::optional<QTreeWidgetItem *>
RQTPerformers::get_performer_line(const std::string & performer_name)
{
  for (int i = 0; i < performers_tree_->topLevelItemCount(); ++i) {
    QTreeWidgetItem * item = performers_tree_->topLevelItem(i);
    if (item->text(0) == QString(performer_name.c_str())) {
      return item;
    }
  }

  return {};
}

void
RQTPerformers::update_performer_row(
  QTreeWidgetItem * row, const plansys2_msgs::msg::ActionPerformerStatus & performer)
{
  switch (performer.state) {
    case plansys2_msgs::msg::ActionPerformerStatus::NOT_READY:
      row->setText(2, QString("NOT_READY"));
      row->setBackground(2, Qt::darkRed);
      break;
    case plansys2_msgs::msg::ActionPerformerStatus::READY:
      row->setText(2, QString("READY"));
      row->setBackground(2, Qt::darkGreen);
      break;
    case plansys2_msgs::msg::ActionPerformerStatus::RUNNING:
      row->setText(2, QString("RUNNING"));
      row->setBackground(2, Qt::yellow);
      break;
    case plansys2_msgs::msg::ActionPerformerStatus::FAILURE:
      row->setText(2, QString("FAILURE"));
      row->setBackground(2, Qt::red);
      break;
  }

  auto elapsed = (node_->now() - performer.status_stamp).seconds();
  row->setText(3, QString::number(elapsed));
  if (elapsed > 5.0) {
    row->setBackground(3, Qt::red);
  } else if (elapsed > 2.0) {
    row->setBackground(3, Qt::yellow);
  }
}

void
RQTPerformers::add_new_row(const plansys2_msgs::msg::ActionPerformerStatus & performer)
{
  auto instance_item = new QTreeWidgetItem();
  instance_item->setText(0, QString(performer.node_name.c_str()));
  instance_item->setText(1, QString(performer.action.c_str()));
  update_performer_row(instance_item, performer);

  std::string join_args;
  for (const auto & arg : performer.specialized_arguments) {
    join_args = join_args + " : " + arg;
  }
  join_args.erase(0, 1);  // remove first ":"

  instance_item->setText(4, QString(join_args.c_str()));

  performers_tree_->addTopLevelItem(instance_item);
}

void
RQTPerformers::spin_loop()
{
  if (!need_update_) {
    return;
  }
  need_update_ = false;

  for (const auto & performer : performers_info_) {
    auto performer_row = get_performer_line(performer.first);

    if (performer_row.has_value()) {
      update_performer_row(performer_row.value(), *performer.second);
    } else {
      add_new_row(*performer.second);
    }
  }
  performers_tree_->parentWidget()->repaint();
}

void
RQTPerformers::restoreSettings(
  const qt_gui_cpp::Settings & plugin_settings, const qt_gui_cpp::Settings & instance_settings)
{
  (void)plugin_settings;
  (void)instance_settings;
}


}  // namespace rqt_plansys2_performers

PLUGINLIB_EXPORT_CLASS(rqt_plansys2_performers::RQTPerformers, rqt_gui_cpp::Plugin)
