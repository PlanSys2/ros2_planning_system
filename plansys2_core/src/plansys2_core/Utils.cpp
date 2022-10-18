// Copyright 2019 Intelligent Robotics Lab
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
#include <sstream>

#include "plansys2_core/Utils.hpp"


namespace plansys2
{

std::vector<std::string> tokenize(const std::string & string, const std::string & delim)
{
  std::string::size_type lastPos = 0, pos = string.find_first_of(delim, lastPos);
  std::vector<std::string> tokens;

  while (lastPos != std::string::npos) {
    if (pos != lastPos) {
      tokens.push_back(string.substr(lastPos, pos - lastPos));
    }
    lastPos = pos;
    if (lastPos == std::string::npos || lastPos + 1 == string.length()) {
      break;
    }
    pos = string.find_first_of(delim, ++lastPos);
  }

  return tokens;
}

std::string substr_without_empty_lines(
  std::string string,
  std::size_t init_pos,
  std::size_t end_pos)
{
  std::stringstream stream_in(string.substr(init_pos, end_pos - init_pos));
  std::stringstream stream_out;
  std::string line;
  bool first = true;
  while (std::getline(stream_in, line)) {
    if (!(line.empty() || line.find_first_not_of(' ') == std::string::npos)) {
      if (first) {
        first = false;
      } else {
        stream_out << "\n";
      }
      stream_out << line;
    }
  }
  return stream_out.str();
}

std::string remove_comments(const std::string & pddl)
{
  std::stringstream uncomment;

  std::size_t pddl_length = pddl.length();
  std::size_t start_pos = 0;
  std::size_t end_pos = 0;
  bool commented = false;
  while (end_pos < pddl_length) {
    if ((!commented) && (pddl.at(end_pos) == ';')) {
      commented = true;
      uncomment << pddl.substr(start_pos, end_pos - start_pos);
    }

    if (commented && (pddl.at(end_pos) == '\r' || pddl.at(end_pos) == '\n')) {
      commented = false;
      start_pos = end_pos;
    }
    end_pos++;
  }
  if (!commented) {
    uncomment << pddl.substr(start_pos, end_pos - start_pos);
  }
  return std::string(uncomment.str());
}

    void encode_plan(const std::shared_ptr<PlanNode>& root, std::vector<int>& struc, std::vector<plansys2_msgs::msg::PlanItem> &data){
        if(root == nullptr){
            struc.push_back(0);
            return;
        }
        struc.push_back(1);
        data.push_back(root->item);
        encode_plan(root->true_node, struc, data);
        encode_plan(root->false_node, struc, data);
    }

    std::shared_ptr<PlanNode> decode_plan(std::vector<int>& struc, std::vector<plansys2_msgs::msg::PlanItem> &data){
        std::queue<int> struc_queue;
        std::queue<plansys2_msgs::msg::PlanItem> data_queue;
        for (const auto& e: data)
            data_queue.push(e);

        for (const auto& e: struc)
            struc_queue.push(e);

        return decode_plan(struc_queue, data_queue);
    }

    std::shared_ptr<PlanNode> decode_plan(std::queue<int>& struc, std::queue<plansys2_msgs::msg::PlanItem> &data){
        if(struc.empty())
            return nullptr;
        bool b = struc.front();
        struc.pop();
        if(b == 1){
            plansys2_msgs::msg::PlanItem key = data.front();
            data.pop();
            auto root = std::make_shared<PlanNode>();
            root->item = key;
            root->true_node = decode_plan(struc, data);
            root->false_node = decode_plan(struc, data);
            return root;
        }
        return nullptr;
    }


}  // namespace plansys2
