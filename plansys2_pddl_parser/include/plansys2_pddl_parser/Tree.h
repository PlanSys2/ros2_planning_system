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

#ifndef PLANSYS2_PDDL_PARSER__TREE_H_
#define PLANSYS2_PDDL_PARSER__TREE_H_

#include <regex>
#include <optional>
#include <string>
#include <vector>
#include <tuple>
#include <memory>
#include <iostream>

namespace parser
{
namespace pddl
{
namespace tree
{

// The type of node
typedef enum {
    AND,
    OR,
    NOT,
    ACTION,
    PREDICATE,
    FUNCTION,
    EXPRESSION,
    FUNCTION_MODIFIER,
    NUMBER,
    UNKNOWN_NODE_TYPE
} NodeType;

// The type of expression
typedef enum {
    COMP_GE,
    COMP_GT,
    COMP_LE,
    COMP_LT,
    ARITH_MULT,
    ARITH_DIV,
    UNKNOWN_EXPR_TYPE
} ExprType;

// The type of function modifier
typedef enum {
    ASSIGN,
    INCREASE,
    DECREASE,
    SCALE_UP,
    SCALE_DOWN,
    UNKNOWN_FUN_MOD_TYPE
} FunModType;

// The type of temporal condition or effect
typedef enum {
    AT_START,
    OVER_ALL,
    AT_END,
    UNKNOWN_TEMPORAL_TYPE
} TemporalType;

// A parameter is defined by a name and a type
struct Param
{
  std::string name;
  std::string type;
  std::vector<std::string> subTypes;
};

/// Test if two parameters are equal
/**
 * \param[in] op1 The first parameter
 * \param[in] op2 The second parameter
 * \return True if name and type are equal.
 */
bool operator==(const Param & op1, const Param & op2);

/// Removes newlines, duplicated spaces, tabs and spaces from parenthesis
/**
 * \param[in] expr The expression to be reduced
 * \return The reduced expression
 */
std::string getReducedString(const std::string & expr);

/// Returns NodeType corresponding to the string input
/**
 * \param[in] input The input string
 * \return The NodeType
 */
NodeType getNodeType(const std::string & expr, NodeType def = UNKNOWN_NODE_TYPE);

/// Returns ExprType corresponding to the string input
/**
 * \param[in] input The input string
 * \return The ExprType
 */
ExprType getExprType(const std::string & input);

/// Returns ExprType and start position of an expression in a string
/**
 * \param[in] input The input string
 * \return result <- tuple(ExprType, int)
 *         result(0) The ExprType
 *         result(1) The start position of the first occurrence
 */
std::tuple < ExprType, int > getExpr(const std::string & input);

/// Returns FunModType corresponding to the string input
/**
 * \param[in] input The input string
 * \return The FunModType
 */
FunModType getFunModType(const std::string & input);

/// Returns FunModType and start position of a function modifier in a string
/**
 * \param[in] input The input string
 * \return result <- tuple(FunModType, int)
 *         result(0) The FunModType
 *         result(1) The start position of the first occurrence
 */
std::tuple < FunModType, int > getFunMod(const std::string & input);

/// A PDDL Predicate
/**
 * This class contains the name and parameters of a predicate.
 */
class Predicate
{
public:
  /// Make a Predicate
  Predicate() {}

  /// Make a Predicate from a string
  /**
    * \param[in] predicate A string containing a predicate
    */
  explicit Predicate(const std::string & predicate)
  {
    fromString(predicate);
  }
  
  /// Generates a string containing the predicate
  /**
   * The resulting string does not contain the type of each parameter; only the name.
   *
   * \return A text representing the predicate (name name_param1 name_par2 ... name_parN)
   */
  std::string toString(bool negate = false) const
  {
    std::string ret;

    if (negate) {
      ret = "(not (" + name;
    } else {
      ret = "(" + name;
    }

    for (const auto & param : parameters) {
      ret += " " + param.name;
    }

    if (negate) {
      ret += "))";
    } else {
      ret += ")";
    }

    return ret;
  }

  /// Generates a Predicate from a string containing the predicate
  /**
   * The input string does not contain the type of each parameter; only the name.
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
  bool negative = {false};
};

/// A PDDL Function
/**
 * This class contains the name, parameters, and value of a function.
 */
class Function
{
public:
  /// Make a Function
  Function() {}

  /// Make a Function from a string
  /**
    * \param[in] function A string containing a function
    */
  explicit Function(const std::string & function)
  {
    fromString(function);
  }

  /// Generates a string containing the function
  /**
   * The resulting string does not contain the type of each parameter; only the name.
   *
   * \return A text representing the function (name name_param1 name_par2 ... name_parN)
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

  /// Generates a Function from a string containing the function
  /**
   * The input string does not contain the type of each parameter; only the name.
   *
   * \param[in] function A string containing a function
   */
  void fromString(const std::string & function)
  {
    std::regex name_regexp("[a-zA-Z][a-zA-Z0-9_\\-]*");
    std::regex number_regexp("[+-]?([0-9]+([.][0-9]*)?|[.][0-9]+)");

    std::smatch match;
    std::string temp = function;

    if (std::regex_search(temp, match, name_regexp)) {
      name = match.str(0);
      temp = match.suffix().str();
    }

    while (std::regex_search(temp, match, name_regexp)) {
      parameters.push_back(Param{match.str(0), ""});
      temp = match.suffix().str();
    }

    if (std::regex_search(temp, match, number_regexp)) {
      value = std::stod(match.str(0));
    }
  }

  friend bool operator==(const Function & op1, const Function & op2);

  std::string name;
  std::vector<Param> parameters;
  double value;
};

/// The base class for nodes in a tree
/**
 * This is the base class for defining a node in a tree.
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
   * \return The string representing this node (and its children in cascade)
   */
  virtual std::string toString(bool negate = false) = 0;

  /// This method will be recursivelly called to recollect the predicates in the tree.
  /**
   * \param[out] predicates Predicates in the node (and its children in cascade)
   */
  virtual void getPredicates(std::vector<Predicate> & predicates, bool negate) = 0;

  NodeType type_;
  bool negate_ {false};
};

/// This function creates a complete tree node.
/**
 * This function recursivelly extracts the logic expressions and predicates from the expression.
 *
 * \param[in] expr A string containg predicates and logic operators
 * \param[in] construct A string containg the associated PDDL constructs
 * \return A smart pointer to the node created
*/
std::shared_ptr<TreeNode> get_tree_node(const std::string & expr, bool negate = false, NodeType parent = UNKNOWN_NODE_TYPE);

/// A node that contains a Predicate
class PredicateNode : public TreeNode
{
public:
  /// Generates a Predicate node
  PredicateNode()
  : TreeNode(PREDICATE) {}

  /// Generates a string from the predicate of this node
  /**
   * \return The string with the predicate
   */
  std::string toString(bool negate = false)
  {
    std::string ret = predicate_.toString(negate);

    return ret;
  }

  /// Adds the containing predicate to the vector of predicates
  /**
   * \param[out] predicates The vector of predicates
   */
  void getPredicates(std::vector<Predicate> & predicates, bool negate)
  {
    negate_ = negate;
    predicate_.negative = negate_;
    predicates.push_back(predicate_);
  }

  Predicate predicate_;
};

/// A node that contains a Function
class FunctionNode : public TreeNode
{
public:
  /// Generates a Function node
  FunctionNode()
  : TreeNode(FUNCTION) {}

  /// Generates a string from the function of this node
  /**
   * \return The string with the function
   */
  std::string toString(bool negate = false)
  {
    return function_.toString();
  }

  /// Required by the superclass, but nothing to do for this subclass
  /**
   * \param[out] predicates The vector of predicates
   */
  void getPredicates(std::vector<Predicate> & predicates, bool only_positives = false)
  {
    // Do nothing
  }

  Function function_;
};

/// A node that contains a Number
class NumberNode : public TreeNode
{
public:
  /// Generates a Number node
  NumberNode()
  : TreeNode(NUMBER) {}

  /// Generates a string from the number of this node
  /**
   * \return The string with the number
   */
  std::string toString(bool negate = false)
  {
    return std::to_string(value_);
  }

  /// Required by the superclass, but nothing to do for this subclass
  /**
   * \param[out] predicates The vector of predicates
   */
  void getPredicates(std::vector<Predicate> & predicates, bool only_positives = false)
  {
    // Do nothing
  }

  double value_;
};

/// A node that contains an And expression
class AndNode : public TreeNode
{
public:
  /// Generates an And node
  AndNode()
  : TreeNode(AND) {}

  ~AndNode() override
  {
  }

  /// Generates a string adding the and logic operator and the child content
  /**
   * \return The string with the and logic operator and the child content
   */
  std::string toString(bool negate = false)
  {
    std::string ret;

    if (!negate) {
      ret = "(and ";
    } else {
      ret = "(or ";
    }

    for (auto op : ops) {
      ret += op->toString(negate);
    }
    ret += ")";

    return ret;
  }

  /// This method calls recursivelly to the getPredicates method of its children.
  /**
   * \param[out] predicates The vector of predicates
   */
  void getPredicates(std::vector<Predicate> & predicates, bool negate)
  {
    negate_ = negate;
    for (auto op : ops) {
      op->getPredicates(predicates, negate_);
    }
  }

  std::vector<std::shared_ptr<TreeNode>> ops;
};

/// A node that contains an Or expression
class OrNode : public TreeNode
{
public:
  /// Generates a Or node
  OrNode()
  : TreeNode(OR) {}

  ~OrNode() override
  {
  }

  /// Generates a string adding the or logic operator and the child content
  /**
   * \return The string with the or logic operator and the child content
   */
  std::string toString(bool negate = false)
  {
    std::string ret;

    if (!negate) {
      ret = "(or ";
    } else {
      ret = "(and ";
    }

    for (auto op : ops) {
      ret += op->toString(negate);
    }
    ret += ")";

    return ret;
  }

  /// This method calls recursivelly to the getPredicates method of its children.
  /**
   * \param[out] predicates The vector of predicates
   */
  void getPredicates(std::vector<Predicate> & predicates, bool negate)
  {
    negate_ = negate;
    for (auto op : ops) {
      op->getPredicates(predicates, negate_);
    }
  }

  std::vector<std::shared_ptr<TreeNode>> ops;
};

/// A node that contains a Not expression
class NotNode : public TreeNode
{
public:
  /// Generates a Not node
  NotNode()
  : TreeNode(NOT), op(nullptr) {}

  ~NotNode() override
  {
  }

  /// Generates a string adding the not logic operator and the child content
  /**
   * \return The string with the not logic operator and the child content
   */
  std::string toString(bool negate = false)
  {
    std::string ret;

    ret += op->toString(!negate);

    return ret;
  }

  /// This method calls recursivelly to the getPredicates method of its children.
  /**
   * \param[out] predicates The vector of predicates
   */
  void getPredicates(std::vector<Predicate> & predicates, bool negate)
  {
    negate_ = negate;
    op->getPredicates(predicates, !negate_);
  }

  std::shared_ptr<TreeNode> op;
};

/// A node that contains a numeric Expression
class ExpressionNode : public TreeNode
{
public:
  /// Generates an Expression node
  ExpressionNode()
  : TreeNode(EXPRESSION) {}

  ~ExpressionNode() override
  {
  }

  /// Generates a string adding the expression logic operator and the child content
  /**
   * \return The string with the expression logic operator and the child content
   */
  std::string toString(bool negate = false)
  {
    std::string ret;

    switch (expr_type) {
    case COMP_GE:
        ret = "(>= ";
        break;
    case COMP_GT:
        ret = "(> ";
        break;
    case COMP_LE:
        ret = "(<= ";
        break;
    case COMP_LT:
        ret = "(< ";
        break;
    case ARITH_MULT:
        ret = "(* ";
        break;
    case ARITH_DIV:
        ret = "(/ ";
        break;
    default:
        break;
    }

    for (auto op : ops) {
      ret += op->toString();
    }
    ret += ")";

    return ret;
  }

  /// Required by the superclass, but nothing to do for this subclass
  /**
   * \param[out] functions The vector of predicates
   */
  void getPredicates(std::vector<Predicate> & predicates, bool only_positives = false)
  {
    // Do nothing
  }

  ExprType expr_type;
  std::vector<std::shared_ptr<TreeNode>> ops;
};

/// A node that contains a Function Modifier expression
class FunctionModifierNode : public TreeNode
{
public:
  /// Generates a Function Modifier node
  FunctionModifierNode()
  : TreeNode(FUNCTION_MODIFIER) {}

  ~FunctionModifierNode() override
  {
  }

  /// Generates a string adding the modifier operator and the child content
  /**
   * \return The string with the modifier operator and the child content
   */
  std::string toString(bool negate = false)
  {
    std::string ret;

    switch (modifier_type) {
    case ASSIGN:
        ret = "(assign ";
        break;
    case INCREASE:
        ret = "(increase ";
        break;
    case DECREASE:
        ret = "(decrease ";
        break;
    case SCALE_UP:
        ret = "(scale-up ";
        break;
    case SCALE_DOWN:
        ret = "(scale-down ";
        break;
    default:
        break;
    }

    for (auto op : ops) {
      ret += op->toString();
    }
    ret += ")";

    return ret;
  }

  /// Required by the superclass, but nothing to do for this subclass
  /**
   * \param[out] functions The vector of predicates with the child's ones
   */
  void getPredicates(std::vector<Predicate> & functions, bool only_positives = false)
  {
    // Do nothing
  }

  FunModType modifier_type;
  std::vector<std::shared_ptr<TreeNode>> ops;
};

/// A node that contains an Action
class ActionNode : public TreeNode
{
public:
  /// Generates an Action node
  ActionNode()
  : TreeNode(ACTION) {}

  ~ActionNode() override
  {
  }

  /// Required by the superclass, but nothing to do for this subclass
  /**
   * \return The string with the predicate
   */
  std::string toString(bool negate = false)
  {
    return std::string("");
  }

  /// This method calls recursivelly to the getPredicates method of its children.
  /**
   * \param[out] predicates The vector of predicates
   */
  void getPredicates(std::vector<Predicate> & predicates, bool only_positives = false)
  {
    for (auto op : pre) {
      op->getPredicates(predicates, only_positives);
    }
    for (auto op : eff) {
      op->getPredicates(predicates, only_positives);
    }
  }

  std::vector<std::shared_ptr<TreeNode>> pre;
  std::vector<std::shared_ptr<TreeNode>> eff;
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
  std::string toString(bool negate = false) const
  {
    if (root_ != nullptr) {
      return root_->toString(negate);
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

  /// This method calls recursivelly to the getPredicates method of its chidren.
  /**
   * \param[out] predicates The vector of predicates contained in the PredicateTree.
   */
  void getPredicates(std::vector<Predicate> & predicates) const
  {
    if (root_ != nullptr) {
      root_->getPredicates(predicates, false);
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

  std::string name_actions_to_string()
  {
    std::string ret = name;
    for (const auto & param : parameters) {
      ret = ret + " " + param.name;
    }
    return ret;
  }
};

struct Instance
{
  std::string name;
  std::string type;
};

typedef PredicateTree Goal;

}  // namespace tree
}  // namespace pddl
}  // namespace parser

 namespace plansys2 {
  typedef parser::pddl::tree::Predicate Predicate;
  typedef parser::pddl::tree::Function Function;
  typedef parser::pddl::tree::Instance Instance;
  typedef parser::pddl::tree::Goal Goal;
}

#endif  // PLANSYS2_PDDL_PARSER__TREE_H_
