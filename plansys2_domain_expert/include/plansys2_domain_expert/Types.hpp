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

#ifndef PLANSYS2_DOMAIN_EXPERT__TYPES_HPP_
#define PLANSYS2_DOMAIN_EXPERT__TYPES_HPP_

#include <boost/optional.hpp>

#include <string>
#include <vector>
#include <memory>
#include <iostream>

namespace plansys2
{

// A parameter is composed by a name and a type
struct Param
{
  std::string name;
  std::string type;
  std::vector<std::string> subTypes;
};

/// Test if two parameters are equals
/**
 * \param[in] op1 The first parameter.
 * \param[in] op2 The second parameter.
 * \return True if name and type are equals.
 */
bool operator==(const Param & op1, const Param & op2);

/// Removes newlines, duplicated spaces, tabs and spaces near from parenthesis from an espression.
/**
 * \param[in] expr The expression to be reduced.
 * \return The expression reduced
 */
std::string getReducedString(const std::string & expr);

/** 
 * @brief A PDDL Function Assignment
 * This class contains the name and parameters of an assignment
 */
class Assignment {
 public:
  /** 
   * @brief Make a Assignment
   */ 
  Assignment() {}

  /**
   * @brief   Make a Assignment from string
   * @param[in] assignment A string containing an assignment
   */
  explicit Assignment(const std::string & assignment) {
    fromString(assignment);
  }

  /**
   * @brief Generates a string containing the assignment
   *      The resulting string does not contains the type of each parameter; only the name.
   *
   * @return A text representing the assignment (= (name name_param1 name_par2 ... name_parN) value)
  */
  std::string toString() const {
    std::string ret;
    ret = "(= (" + name;
    for (const auto & param : parameters) {
      ret += " " + param.name;
    }
    ret += ") " + std::to_string(value) + ")";

    return ret;
  }

  /**
   * @brief Check for the identical name and paramters.
   * 
   * @param other the assignment to compare
   * @return true if the name and the parameters of the assignments are the same. Even if the value differs.
   * @return false in other cases
   */
  bool hasSameNamesAndParameters(const Assignment & other);

  /**
   * @brief split a "lisp -like" expression.
   *    Remove the spaces and the first bracket at both ends when ballanded.
   * 
   * @param[in] expression a bracketed e
   * @return a vector with the first level elements in the expression
   */
  std::vector<std::string> splitExpr(const std::string & expression);

  /**
   * /brief Generates a Assignment from a string containing the assignment
   * The resulting string does not contains the type of each parameter; only the name.
   *  (= (name name_param1 name_par2 ... name_parN) value)
   *
   * \param[in] assignment A string containing a assignment
   */
  void fromString(const std::string & assignment) {
    std::vector<std::string> subexprs = splitExpr(assignment);

    // subexprs[0] should be "=";

    std::vector<std::string> subexprsFunc = splitExpr(subexprs[1]);
    name = subexprsFunc[0];
    for (size_t i = 1; i < subexprsFunc.size(); i++) {
      parameters.push_back(Param{subexprsFunc[i], ""});
    }

    value = atof(subexprs[2].c_str());
  }

  friend bool operator==(const Assignment & op1, const Assignment & op2);

  std::string name;
  std::vector<Param> parameters;
  double value;
};


class Function {
 public:
 /**
  * @brief Construct a new Function object
  * 
  */
  Function() {}  // TODO(Fabrice) : Is this usefull ?

 public:
  std::string name;
  std::vector<Param> parameters;
};

/// A PDDL Predicate
/**
 * This class contains the name and parameters of a predicate
 */
class Predicate
{
public:
  /// Make a Predicate
  Predicate() {}

  /// Make a Predicate from string
  /**
    * \param[in] predicate A string containing a predicate
    */
  explicit Predicate(const std::string & predicate)
  {
    fromString(predicate);
  }
  /// Generates a string containing the predicate
  /**
   * The resulting string does not contains the type of each parameter; only the name.
   *
   * \return A text representing the predicate (name name_param1 name_par2 ... name_parN)
  */
  std::string toString() const
  {
    std::string ret;
    ret = "(" + name;
    for (const auto & param : parameters) {
      ret += " " + param.name;
    }

    ret += ")";

    return ret;
  }

  /// Generates a Predicate from a string containing the predicate
  /**
   * The resulting string does not contains the type of each parameter; only the name.
   *
   * \param[in] predicate A string containing a predicate
   */
  void fromString(const std::string & predicate)
  {
    std::vector<std::string> tokens;
    size_t start = 0, end = 0;

    while (end != std::string::npos) {
      end = predicate.find(" ", start);
      tokens.push_back(
        predicate.substr(
          start,
          (end == std::string::npos) ? std::string::npos : end - start));
      start = ((end > (std::string::npos - 1)) ? std::string::npos : end + 1);
    }

    tokens[0].erase(0, 1);
    name = tokens[0];

    tokens.back().pop_back();

    for (size_t i = 1; i < tokens.size(); i++) {
      parameters.push_back(Param{tokens[i], ""});
    }
  }

  friend bool operator==(const Predicate & op1, const Predicate & op2);

  std::string name;
  std::vector<Param> parameters;
};

// The type of nodes in a TreeNode
typedef enum {AND, OR, NOT, PREDICATE, UNKNOWN} NodeType;

/// The base class for nodes in a tree of PDDL Nodes.
/**
 * This class is created to analyze and to evaluate logical PDDL expression
 */
class TreeNode
{
public:
  /// Generates a tree node
  /**
   * \param[in] The type of the node
   */
  explicit TreeNode(NodeType type)
  : type_(type) {}
  virtual ~TreeNode() {}

  /// Generates a string from this node
  /**
   * \return The string representing this node (and its childs in cascade)
   */
  virtual std::string toString() = 0;


  /// This method will be recursively called to recollect the predicates in the tree
  /**
   * \param[out] predicates Predicates in the node (and its childs in cascade)
   */
  virtual void getPredicates(std::vector<Predicate> & predicates) = 0;
  NodeType type_;
};

/// This function creates a complete tree node
/**
 * This function extracts recursivelly the logic expressions and predicates from the expression.
 *
 * \param[in] expr A expression containing predicates and logic operators
 * \return A smart pointer to the node created
*/
std::shared_ptr<TreeNode> get_tree_node(const std::string & expr);

/// A Node that contains a Predicate
class PredicateNode : public TreeNode
{
public:
  /// Generates a Predicate Node
  PredicateNode()
  : TreeNode(PREDICATE) {}

  /// Generates a string from the predicate of this node
  /**
   * \return The string with the predicate
   */
  std::string toString()
  {
    return predicate_.toString();
  }

  /// This method add the containing predicate to the recollected vector of predicates.
  /**
   * \param[out] predicates The vector of predicates with this node's predicate
   */
  void getPredicates(std::vector<Predicate> & predicates)
  {
    predicates.push_back(predicate_);
  }

  Predicate predicate_;
};

/// A Node that contains a And node of Predicates
class AndNode : public TreeNode
{
public:
  /// Generates a And Node
  AndNode()
  : TreeNode(AND) {}

  ~AndNode() override
  {
  }

  /// Generates a string adding the and logic operator and the child content
  /**
   * \return The string with the predicate
   */
  std::string toString()
  {
    std::string ret;
    ret = "(and ";
    for (auto op : ops) {
      ret += op->toString();
    }
    ret += ")";

    return ret;
  }

  /// This method calls recursivelly to the getPredicates method of its childs.
  /**
   * \param[out] predicates The vector of predicates with the child's ones
   */
  void getPredicates(std::vector<Predicate> & predicates)
  {
    for (auto op : ops) {
      op->getPredicates(predicates);
    }
  }

  std::vector<std::shared_ptr<TreeNode>> ops;
};

/// A Node that contains a Or node of Predicates
class OrNode : public TreeNode
{
public:
  /// Generates a Or Node
  OrNode()
  : TreeNode(OR) {}

  ~OrNode() override
  {
  }

  /// Generates a string adding the or logic operator and the child content
  /**
   * \return The string with the predicate
   */
  std::string toString()
  {
    std::string ret;
    ret = "(or ";
    for (auto op : ops) {
      ret += op->toString();
    }
    ret += ")";

    return ret;
  }

  /// This method calls recursivelly to the getPredicates method of its childs.
  /**
   * \param[out] predicates The vector of predicates with the child's ones
   */
  void getPredicates(std::vector<Predicate> & predicates)
  {
    for (auto op : ops) {
      op->getPredicates(predicates);
    }
  }

  std::vector<std::shared_ptr<TreeNode>> ops;
};

/// A Node that contains a Not node of Predicates
class NotNode : public TreeNode
{
public:
  /// Generates a Not Node
  NotNode()
  : TreeNode(NOT), op(nullptr) {}

  ~NotNode() override
  {
  }

  /// Generates a string adding the not logic operator and the child content
  /**
   * \return The string with the predicate
   */
  std::string toString()
  {
    std::string ret;
    ret = "(not ";
    ret += op->toString();
    ret += ")";

    return ret;
  }

  /// This method calls recursivelly to the getPredicates method of its childs.
  /**
   * \param[out] predicates The vector of predicates with the child's ones
   */
  void getPredicates(std::vector<Predicate> & predicates)
  {
    op->getPredicates(predicates);
  }
  std::shared_ptr<TreeNode> op;
};

/// A PredicateTree contains a shared pointer to a tree of nodes
class PredicateTree
{
public:
  /// Generates a PredicateTree
  PredicateTree()
  : root_(nullptr) {}

  explicit PredicateTree(const std::string & predicate)
  : PredicateTree()
  {
    fromString(predicate);
  }

  ~PredicateTree()
  {
  }

  // Reset the PredicateTree
  void clear()
  {
    root_ = nullptr;
  }


  /// Init the tree from other PredicateTree
  /**
   * Take into account that this method does not perform a deep copy
   *
   * \param[in] other Other PredicateTree
   * \return A reference to this PredicateTree
   */
  PredicateTree & operator=(const PredicateTree & other)
  {
    root_ = other.root_;
    return *this;
  }

  /// Get the string that represents this PredicateTree
  /**
   * \return The string with the PDDL logical expression contained in this PredicateTree.
   *    It returns a void string if the PredicateTree is void.
   */
  std::string toString() const
  {
    if (root_ != nullptr) {
      return root_->toString();
    } else {
      return "";
    }
  }

  /// Init this PredicateTree from a string
  /**
   * \param[in] expr A string with a PDDL logical expression
   */
  void fromString(const std::string & expr)
  {
    if (expr == "") {
      root_ = nullptr;
    } else {
      root_ = get_tree_node(expr);
    }
  }

  /// This method calls recursivelly to the getPredicates method of its childs.
  /**
   * \param[out] predicates The vector of predicates contained in the PredicateTree.
   */
  void getPredicates(std::vector<Predicate> & predicates)
  {
    if (root_ != nullptr) {
      root_->getPredicates(predicates);
    }
  }

  /// Get if the PredicateTree is empty
  /**
   * \return If the PredicateTree is empty.
   */
  bool empty()
  {
    return root_ == nullptr || root_->toString() == "(and )";
  }

  std::shared_ptr<TreeNode> root_;
};

/// The Action struct contains all the information of a regular action
struct Action
{
  std::string name;
  std::vector<Param> parameters;
  PredicateTree preconditions;
  PredicateTree effects;
};

/// The DurativeAction struct contains all the information of a temporal action
struct DurativeAction
{
  std::string name;
  std::vector<Param> parameters;
  PredicateTree at_start_requirements;
  PredicateTree over_all_requirements;
  PredicateTree at_end_requirements;
  PredicateTree at_start_effects;
  PredicateTree at_end_effects;
};

}  // namespace plansys2

#endif  // PLANSYS2_DOMAIN_EXPERT__TYPES_HPP_
