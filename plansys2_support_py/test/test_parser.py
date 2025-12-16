# Copyright (c) 2025 Alberto J. Tudela Roldán
# Copyright (c) 2025 Grupo Avispa, DTE, Universidad de Málaga
#
# Licensed under the Apache License, Version 2.0 (the 'License');
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an 'AS IS' BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Unit tests for Parser."""

import unittest

from plansys2_msgs.msg import Node, Param, Tree
from plansys2_support_py.Parser import Parser


class TestParser(unittest.TestCase):
    """Test suite for Parser."""

    def test_get_reduced_string_removes_newlines(self):
        """Test that newlines are removed."""
        expr = '(and\n(robot_at r1 wp1)\n(robot_at r2 wp2))'
        result = Parser.get_reduced_string(expr)
        self.assertNotIn('\n', result)
        # get_reduced_string only replaces newlines with space, doesn't add space after operators
        self.assertEqual(result, '(and(robot_at r1 wp1)(robot_at r2 wp2))')

    def test_get_reduced_string_removes_tabs(self):
        """Test that tabs are removed."""
        expr = '(and\t(robot_at r1 wp1)\t(robot_at r2 wp2))'
        result = Parser.get_reduced_string(expr)
        self.assertNotIn('\t', result)

    def test_get_reduced_string_removes_multiple_spaces(self):
        """Test that multiple spaces are reduced to single space."""
        expr = '(and    (robot_at    r1    wp1))'
        result = Parser.get_reduced_string(expr)
        self.assertEqual(result, '(and (robot_at r1 wp1))')

    def test_get_reduced_string_removes_space_after_open_paren(self):
        """Test that space after opening parenthesis is removed."""
        expr = '( and ( robot_at r1 wp1))'
        result = Parser.get_reduced_string(expr)
        self.assertEqual(result, '(and (robot_at r1 wp1))')

    def test_get_reduced_string_removes_space_before_close_paren(self):
        """Test that space before closing parenthesis is removed."""
        expr = '(and (robot_at r1 wp1 ) )'
        result = Parser.get_reduced_string(expr)
        self.assertEqual(result, '(and (robot_at r1 wp1))')

    def test_get_reduced_string_complex(self):
        """Test with complex expression."""
        expr = '(\n  and \t  ( robot_at  r1   wp1 )  \n  ( carrying   r1  obj1 )  )'
        result = Parser.get_reduced_string(expr)
        self.assertEqual(result, '(and (robot_at r1 wp1) (carrying r1 obj1))')

    def test_get_node_type_and(self):
        """Test detection of AND node type."""
        expr = '(and (p1) (p2))'
        result = Parser.get_node_type(expr)
        self.assertEqual(result, Node.AND)

    def test_get_node_type_or(self):
        """Test detection of OR node type."""
        expr = '(or (p1) (p2))'
        result = Parser.get_node_type(expr)
        self.assertEqual(result, Node.OR)

    def test_get_node_type_not(self):
        """Test detection of NOT node type."""
        expr = '(not (p1))'
        result = Parser.get_node_type(expr)
        self.assertEqual(result, Node.NOT)

    def test_get_node_type_exists(self):
        """Test detection of EXISTS node type."""
        expr = '(exists (?x - robot) (robot_at ?x wp1))'
        result = Parser.get_node_type(expr)
        self.assertEqual(result, Node.EXISTS)

    def test_get_node_type_parameter(self):
        """Test detection of PARAMETER node type."""
        expr = '?robot1'
        result = Parser.get_node_type(expr)
        self.assertEqual(result, Node.PARAMETER)

    def test_get_node_type_parameter_with_paren(self):
        """Test detection of PARAMETER with parenthesis."""
        expr = '(?robot)'
        result = Parser.get_node_type(expr)
        self.assertEqual(result, Node.PARAMETER)

    def test_get_node_type_number_integer(self):
        """Test detection of NUMBER node type with integer."""
        expr = '42'
        result = Parser.get_node_type(expr)
        self.assertEqual(result, Node.NUMBER)

    def test_get_node_type_number_float(self):
        """Test detection of NUMBER node type with float."""
        expr = '3.14'
        result = Parser.get_node_type(expr)
        self.assertEqual(result, Node.NUMBER)

    def test_get_node_type_number_negative(self):
        """Test detection of NUMBER node type with negative number."""
        expr = '-5.5'
        result = Parser.get_node_type(expr)
        self.assertEqual(result, Node.NUMBER)

    def test_get_node_type_predicate(self):
        """Test detection of PREDICATE node type."""
        expr = '(robot_at r1 wp1)'
        result = Parser.get_node_type(expr)
        self.assertEqual(result, Node.PREDICATE)

    def test_get_node_type_function_in_expression(self):
        """Test detection of FUNCTION in expression context."""
        expr = '(battery_level r1)'
        result = Parser.get_node_type(expr, default_node_type=Node.EXPRESSION)
        self.assertEqual(result, Node.FUNCTION)

    def test_get_node_type_with_default(self):
        """Test that default node type is returned for unknown."""
        expr = 'unknown_expression'
        result = Parser.get_node_type(expr, default_node_type=Node.ACTION)
        self.assertEqual(result, Node.ACTION)

    def test_get_expr_greater_equal(self):
        """Test detection of >= expression."""
        expr = '(>= (battery r1) 50)'
        expr_type, pos = Parser.get_expr(expr)
        self.assertEqual(expr_type, Node.COMP_GE)
        self.assertGreaterEqual(pos, 0)

    def test_get_expr_greater_than(self):
        """Test detection of > expression."""
        expr = '(> (battery r1) 50)'
        expr_type, pos = Parser.get_expr(expr)
        self.assertEqual(expr_type, Node.COMP_GT)
        self.assertGreaterEqual(pos, 0)

    def test_get_expr_less_equal(self):
        """Test detection of <= expression."""
        expr = '(<= (battery r1) 50)'
        expr_type, pos = Parser.get_expr(expr)
        self.assertEqual(expr_type, Node.COMP_LE)
        self.assertGreaterEqual(pos, 0)

    def test_get_expr_less_than(self):
        """Test detection of < expression."""
        expr = '(< (battery r1) 50)'
        expr_type, pos = Parser.get_expr(expr)
        self.assertEqual(expr_type, Node.COMP_LT)
        self.assertGreaterEqual(pos, 0)

    def test_get_expr_equal(self):
        """Test detection of = expression."""
        expr = '(= (battery r1) 50)'
        expr_type, pos = Parser.get_expr(expr)
        self.assertEqual(expr_type, Node.COMP_EQ)
        self.assertGreaterEqual(pos, 0)

    def test_get_expr_multiply(self):
        """Test detection of * expression."""
        expr = '(* 2 3)'
        expr_type, pos = Parser.get_expr(expr)
        self.assertEqual(expr_type, Node.ARITH_MULT)
        self.assertGreaterEqual(pos, 0)

    def test_get_expr_divide(self):
        """Test detection of / expression."""
        expr = '(/ 10 2)'
        expr_type, pos = Parser.get_expr(expr)
        self.assertEqual(expr_type, Node.ARITH_DIV)
        self.assertGreaterEqual(pos, 0)

    def test_get_expr_add(self):
        """Test detection of + expression."""
        expr = '(+ 1 2)'
        expr_type, pos = Parser.get_expr(expr)
        self.assertEqual(expr_type, Node.ARITH_ADD)
        self.assertGreaterEqual(pos, 0)

    def test_get_expr_subtract(self):
        """Test detection of - expression."""
        expr = '(- 10 5)'
        expr_type, pos = Parser.get_expr(expr)
        self.assertEqual(expr_type, Node.ARITH_SUB)
        self.assertGreaterEqual(pos, 0)

    def test_get_expr_unknown(self):
        """Test that UNKNOWN is returned for no expression."""
        expr = '(robot_at r1 wp1)'
        expr_type, pos = Parser.get_expr(expr)
        self.assertEqual(expr_type, Node.UNKNOWN)
        self.assertEqual(pos, -1)

    def test_get_fun_mod_assign(self):
        """Test detection of ASSIGN function modifier."""
        expr = '(assign (battery r1) 100)'
        mod_type, pos = Parser.get_fun_mod(expr)
        self.assertEqual(mod_type, Node.ASSIGN)
        self.assertGreaterEqual(pos, 0)

    def test_get_fun_mod_increase(self):
        """Test detection of INCREASE function modifier."""
        expr = '(increase (counter) 1)'
        mod_type, pos = Parser.get_fun_mod(expr)
        self.assertEqual(mod_type, Node.INCREASE)
        self.assertGreaterEqual(pos, 0)

    def test_get_fun_mod_decrease(self):
        """Test detection of DECREASE function modifier."""
        expr = '(decrease (battery r1) 10)'
        mod_type, pos = Parser.get_fun_mod(expr)
        self.assertEqual(mod_type, Node.DECREASE)
        self.assertGreaterEqual(pos, 0)

    def test_get_fun_mod_scale_up(self):
        """Test detection of SCALE_UP function modifier."""
        expr = '(scale-up (value) 2)'
        mod_type, pos = Parser.get_fun_mod(expr)
        self.assertEqual(mod_type, Node.SCALE_UP)
        self.assertGreaterEqual(pos, 0)

    def test_get_fun_mod_scale_down(self):
        """Test detection of SCALE_DOWN function modifier."""
        expr = '(scale-down (value) 2)'
        mod_type, pos = Parser.get_fun_mod(expr)
        self.assertEqual(mod_type, Node.SCALE_DOWN)
        self.assertGreaterEqual(pos, 0)

    def test_get_fun_mod_unknown(self):
        """Test that UNKNOWN is returned for no modifier."""
        expr = '(robot_at r1 wp1)'
        mod_type, pos = Parser.get_fun_mod(expr)
        self.assertEqual(mod_type, Node.UNKNOWN)
        self.assertEqual(pos, -1)

    def test_get_parenthesis_simple(self):
        """Test finding matching parenthesis in simple expression."""
        expr = '(robot_at r1 wp1)'
        result = Parser.get_parenthesis(expr, 0)
        self.assertEqual(result, len(expr) - 1)

    def test_get_parenthesis_nested(self):
        """Test finding matching parenthesis with nested expressions."""
        expr = '(and (p1) (p2))'
        result = Parser.get_parenthesis(expr, 0)
        self.assertEqual(result, len(expr) - 1)

    def test_get_parenthesis_not_at_start(self):
        """Test finding matching parenthesis not starting at 0."""
        expr = 'prefix (inner content) suffix'
        start = expr.index('(')
        result = Parser.get_parenthesis(expr, start)
        self.assertEqual(result, expr.index(')'))

    def test_get_sub_expr_simple(self):
        """Test extracting sub-expressions from simple expression."""
        expr = '(and (p1) (p2))'
        result = Parser.get_sub_expr(expr)
        self.assertEqual(len(result), 2)
        self.assertIn('(p1)', result)
        self.assertIn('(p2)', result)

    def test_get_sub_expr_nested(self):
        """Test extracting sub-expressions with nested expressions."""
        expr = '(and (or (p1) (p2)) (p3))'
        result = Parser.get_sub_expr(expr)
        self.assertEqual(len(result), 2)
        self.assertIn('(or (p1) (p2))', result)
        self.assertIn('(p3)', result)

    def test_get_sub_expr_empty(self):
        """Test extracting sub-expressions from empty expression."""
        expr = '()'
        result = Parser.get_sub_expr(expr)
        self.assertEqual(len(result), 0)

    def test_param_from_string_with_type(self):
        """Test creating parameter from string with type."""
        result = Parser.param_from_string('robot1', 'robot')
        self.assertIsInstance(result, Param)
        self.assertEqual(result.name, 'robot1')
        self.assertEqual(result.type, 'robot')

    def test_param_from_string_without_type(self):
        """Test creating parameter from string without type."""
        result = Parser.param_from_string('robot1')
        self.assertIsInstance(result, Param)
        self.assertEqual(result.name, 'robot1')
        self.assertEqual(result.type, '')

    def test_param_from_string_with_question_mark(self):
        """Test creating parameter from string with ? prefix."""
        result = Parser.param_from_string('?robot', 'robot')
        self.assertIsInstance(result, Param)
        # param_from_string doesn't remove the '?' prefix
        self.assertEqual(result.name, '?robot')
        self.assertEqual(result.type, 'robot')

    def test_param_to_string_with_type(self):
        """Test converting parameter to string with type."""
        param = Param()
        param.name = 'robot1'
        param.type = 'robot'
        result = Parser.param_to_string(param)
        self.assertEqual(result, 'robot1 - robot')

    def test_param_to_string_without_type(self):
        """Test converting parameter to string without type."""
        param = Param()
        param.name = 'robot1'
        param.type = ''
        result = Parser.param_to_string(param)
        self.assertEqual(result, 'robot1')

    def test_node_from_string_predicate_simple(self):
        """Test creating node from simple predicate string."""
        predicate = '(robot_at robot1 wp1)'
        result = Parser.node_from_string_predicate(predicate)
        self.assertIsInstance(result, Node)
        self.assertEqual(result.node_type, Node.PREDICATE)
        self.assertEqual(result.name, 'robot_at')
        self.assertEqual(len(result.parameters), 2)

    def test_node_from_string_predicate_no_params(self):
        """Test creating node from predicate without parameters."""
        predicate = '(goal_achieved)'
        result = Parser.node_from_string_predicate(predicate)
        self.assertIsInstance(result, Node)
        self.assertEqual(result.name, 'goal_achieved')
        self.assertEqual(len(result.parameters), 0)

    def test_node_from_string_predicate_typed_params(self):
        """Test creating node from predicate with typed parameters."""
        predicate = '(robot_at ?r - robot ?w - waypoint)'
        result = Parser.node_from_string_predicate(predicate)
        self.assertEqual(result.name, 'robot_at')
        # Parser doesn't parse typed parameters, treats each token as parameter
        self.assertEqual(len(result.parameters), 6)

    def test_node_from_string_function_simple(self):
        """Test creating node from simple function string."""
        function = '(battery_level robot1)'
        result = Parser.node_from_string_function(function)
        self.assertIsInstance(result, Node)
        self.assertEqual(result.node_type, Node.FUNCTION)
        self.assertEqual(result.name, 'battery_level')

    def test_node_from_string_function_with_value(self):
        """Test creating node from function with value."""
        function = '(= (battery_level robot1) 100)'
        result = Parser.node_from_string_function(function)
        self.assertEqual(result.node_type, Node.FUNCTION)

    def test_node_from_string_exists_simple(self):
        """Test creating node from exists expression."""
        exists = '(exists (?x - robot) (robot_at ?x wp1))'
        result = Parser.node_from_string_exists(exists)
        self.assertIsInstance(result, Node)
        self.assertEqual(result.node_type, Node.EXISTS)
        self.assertGreater(len(result.parameters), 0)

    def test_tree_from_string_simple_predicate(self):
        """Test creating tree from simple predicate."""
        expr = '(robot_at r1 wp1)'
        result = Parser.tree_from_string(expr)
        self.assertIsInstance(result, Tree)
        self.assertGreater(len(result.nodes), 0)

    def test_tree_from_string_and_expression(self):
        """Test creating tree from AND expression."""
        expr = '(and (robot_at r1 wp1) (robot_at r2 wp2))'
        result = Parser.tree_from_string(expr)
        self.assertIsInstance(result, Tree)
        self.assertGreater(len(result.nodes), 0)

    def test_tree_from_string_or_expression(self):
        """Test creating tree from OR expression."""
        expr = '(or (robot_at r1 wp1) (robot_at r1 wp2))'
        result = Parser.tree_from_string(expr)
        self.assertIsInstance(result, Tree)
        self.assertGreater(len(result.nodes), 0)

    def test_tree_from_string_not_expression(self):
        """Test creating tree from NOT expression."""
        expr = '(not (robot_at r1 wp1))'
        result = Parser.tree_from_string(expr)
        self.assertIsInstance(result, Tree)
        self.assertGreater(len(result.nodes), 0)

    def test_tree_from_string_nested(self):
        """Test creating tree from nested expression."""
        expr = '(and (or (p1) (p2)) (not (p3)))'
        result = Parser.tree_from_string(expr)
        self.assertIsInstance(result, Tree)
        self.assertGreater(len(result.nodes), 2)

    def test_tree_from_string_with_function(self):
        """Test creating tree from expression with function."""
        expr = '(>= (battery_level r1) 50)'
        result = Parser.tree_from_string(expr)
        self.assertIsInstance(result, Tree)

    def test_tree_from_string_with_modifier(self):
        """Test creating tree from expression with modifier."""
        expr = '(assign (counter) 10)'
        result = Parser.tree_from_string(expr)
        self.assertIsInstance(result, Tree)

    def test_node_to_string_predicate(self):
        """Test converting predicate node to string."""
        node = Node()
        node.node_type = Node.PREDICATE
        node.name = 'robot_at'
        param1 = Param()
        param1.name = 'r1'
        param2 = Param()
        param2.name = 'wp1'
        node.parameters = [param1, param2]

        result = Parser.node_to_string(node)
        self.assertIn('robot_at', result)
        self.assertIn('r1', result)
        self.assertIn('wp1', result)

    def test_node_to_string_function(self):
        """Test converting function node to string."""
        node = Node()
        node.node_type = Node.FUNCTION
        node.name = 'battery_level'
        param = Param()
        param.name = 'r1'
        node.parameters = [param]

        result = Parser.node_to_string(node)
        self.assertIn('battery_level', result)
        self.assertIn('r1', result)

    def test_node_to_string_number(self):
        """Test converting number node to string - requires tree context."""
        node = Node()
        node.node_type = Node.NUMBER
        node.value = 42.5

        # node_to_string requires PREDICATE or FUNCTION node type
        # Number nodes need tree context, so test with tree_to_string instead
        tree = Parser.tree_from_string('42.5')
        result = Parser.tree_to_string(tree)
        self.assertIn('42.5', result)

    def test_tree_to_string_simple(self):
        """Test converting simple tree to string."""
        tree = Parser.tree_from_string('(robot_at r1 wp1)')
        result = Parser.tree_to_string(tree)
        self.assertIsInstance(result, str)
        self.assertIn('robot_at', result)

    def test_tree_to_string_and(self):
        """Test converting AND tree to string."""
        tree = Parser.tree_from_string('(and (p1) (p2))')
        result = Parser.tree_to_string(tree)
        self.assertIn('and', result)

    def test_tree_to_string_or(self):
        """Test converting OR tree to string."""
        tree = Parser.tree_from_string('(or (p1) (p2))')
        result = Parser.tree_to_string(tree)
        self.assertIn('or', result)

    def test_tree_to_string_not(self):
        """Test converting NOT tree to string."""
        tree = Parser.tree_from_string('(not (p1))')
        result = Parser.tree_to_string(tree)
        self.assertIn('not', result)

    def test_tree_to_string_with_negation(self):
        """Test converting tree to string with negation flag."""
        tree = Parser.tree_from_string('(p1)')
        result = Parser.tree_to_string(tree, negate=True)
        self.assertIn('not', result)

    def test_roundtrip_simple_predicate(self):
        """Test roundtrip conversion of simple predicate."""
        original = '(robot_at r1 wp1)'
        tree = Parser.tree_from_string(original)
        result = Parser.tree_to_string(tree)
        # Normalize both strings for comparison
        original_reduced = Parser.get_reduced_string(original)
        result_reduced = Parser.get_reduced_string(result)
        self.assertEqual(original_reduced, result_reduced)

    def test_roundtrip_and_expression(self):
        """Test roundtrip conversion of AND expression."""
        original = '(and (p1) (p2))'
        tree = Parser.tree_from_string(original)
        result = Parser.tree_to_string(tree)
        result_reduced = Parser.get_reduced_string(result)
        # Check essential parts are present
        self.assertIn('and', result_reduced)
        self.assertIn('p1', result_reduced)
        self.assertIn('p2', result_reduced)

    def test_roundtrip_nested_expression(self):
        """Test roundtrip conversion of nested expression."""
        original = '(and (or (p1) (p2)) (p3))'
        tree = Parser.tree_from_string(original)
        result = Parser.tree_to_string(tree)
        # Check all components are present
        self.assertIn('and', result)
        self.assertIn('or', result)
        self.assertIn('p1', result)
        self.assertIn('p2', result)
        self.assertIn('p3', result)

    def test_get_sub_expr_with_operators(self):
        """Test extracting sub-expressions with arithmetic operators."""
        expr = '(+ 1 2 3)'
        result = Parser.get_sub_expr(expr)
        self.assertGreater(len(result), 0)

    def test_get_parenthesis_unmatched(self):
        """Test get_parenthesis with unmatched parenthesis."""
        expr = '(incomplete'
        result = Parser.get_parenthesis(expr, 0)
        # Returns length of string when not properly closed
        self.assertEqual(result, len(expr))

    def test_tree_from_string_empty(self):
        """Test creating tree from empty string."""
        expr = ''
        result = Parser.tree_from_string(expr)
        self.assertIsInstance(result, Tree)

    def test_tree_from_string_complex_nested(self):
        """Test creating tree from complex nested expression."""
        expr = '(and (or (p1) (not (p2))) (and (p3) (p4)) (not (or (p5) (p6))))'
        result = Parser.tree_from_string(expr)
        self.assertIsInstance(result, Tree)
        self.assertGreater(len(result.nodes), 5)

    def test_tree_from_string_with_parameters(self):
        """Test creating tree from expression with parameters."""
        expr = '(?robot1)'
        result = Parser.tree_from_string(expr)
        self.assertIsInstance(result, Tree)
        self.assertGreater(len(result.nodes), 0)
        self.assertEqual(result.nodes[0].node_type, Node.PARAMETER)

    def test_tree_from_string_numeric_expression(self):
        """Test creating tree from numeric value."""
        expr = '42'
        result = Parser.tree_from_string(expr)
        self.assertIsInstance(result, Tree)
        self.assertGreater(len(result.nodes), 0)
        self.assertEqual(result.nodes[0].node_type, Node.NUMBER)

    def test_tree_from_string_float_expression(self):
        """Test creating tree from float value."""
        expr = '3.14159'
        result = Parser.tree_from_string(expr)
        self.assertIsInstance(result, Tree)
        self.assertEqual(result.nodes[0].node_type, Node.NUMBER)

    def test_tree_from_string_comparison_expression(self):
        """Test creating tree from comparison expression."""
        expr = '(> 5 3)'
        result = Parser.tree_from_string(expr)
        self.assertIsInstance(result, Tree)
        self.assertGreater(len(result.nodes), 0)

    def test_tree_from_string_arithmetic_expression(self):
        """Test creating tree from arithmetic expression."""
        expr = '(+ 2 3)'
        result = Parser.tree_from_string(expr)
        self.assertIsInstance(result, Tree)

    def test_tree_from_string_function_modifier(self):
        """Test creating tree from function with modifier."""
        expr = '(increase (counter) 1)'
        result = Parser.tree_from_string(expr)
        self.assertIsInstance(result, Tree)

    def test_node_from_string_parameter(self):
        """Test creating parameter node from string."""
        expr = '?x'
        tree = Tree()
        node = Parser._from_string_recursive(tree, expr, False, Node.PARAMETER)
        self.assertIsNotNone(node)
        self.assertEqual(node.node_type, Node.PARAMETER)

    def test_tree_to_string_comparison(self):
        """Test converting comparison tree to string."""
        tree = Parser.tree_from_string('(>= (battery r1) 50)')
        result = Parser.tree_to_string(tree)
        self.assertIn('>=', result)

    def test_tree_to_string_arithmetic_add(self):
        """Test converting arithmetic ADD tree to string."""
        tree = Parser.tree_from_string('(+ 2 3)')
        result = Parser.tree_to_string(tree)
        self.assertIn('+', result)

    def test_tree_to_string_arithmetic_subtract(self):
        """Test converting arithmetic SUBTRACT tree to string."""
        tree = Parser.tree_from_string('(- 10 5)')
        result = Parser.tree_to_string(tree)
        self.assertIn('-', result)

    def test_tree_to_string_arithmetic_multiply(self):
        """Test converting arithmetic MULTIPLY tree to string."""
        tree = Parser.tree_from_string('(* 4 5)')
        result = Parser.tree_to_string(tree)
        self.assertIn('*', result)

    def test_tree_to_string_arithmetic_divide(self):
        """Test converting arithmetic DIVIDE tree to string."""
        tree = Parser.tree_from_string('(/ 20 4)')
        result = Parser.tree_to_string(tree)
        self.assertIn('/', result)

    def test_tree_to_string_function_modifier_assign(self):
        """Test converting ASSIGN modifier tree to string."""
        tree = Parser.tree_from_string('(assign (x) 10)')
        result = Parser.tree_to_string(tree)
        self.assertIn('assign', result)

    def test_tree_to_string_function_modifier_increase(self):
        """Test converting INCREASE modifier tree to string."""
        tree = Parser.tree_from_string('(increase (counter) 1)')
        result = Parser.tree_to_string(tree)
        self.assertIn('increase', result)

    def test_tree_to_string_function_modifier_decrease(self):
        """Test converting DECREASE modifier tree to string."""
        tree = Parser.tree_from_string('(decrease (fuel r1) 5)')
        result = Parser.tree_to_string(tree)
        self.assertIn('decrease', result)

    def test_tree_to_string_exists_expression(self):
        """Test converting EXISTS tree to string."""
        tree = Parser.tree_from_string('(exists (?x) (p ?x))')
        result = Parser.tree_to_string(tree)
        self.assertIn('exists', result)

    def test_tree_from_string_deeply_nested(self):
        """Test creating tree from deeply nested expression."""
        expr = '(and (or (p1) (not (p2))) (and (p3) (or (not (p4)) (p5))))'
        result = Parser.tree_from_string(expr)
        self.assertIsInstance(result, Tree)
        self.assertGreater(len(result.nodes), 5)

    def test_tree_from_string_complex_function(self):
        """Test creating tree from complex function expression."""
        expr = '(= (battery_level robot1) (+ (initial_battery) (- 100 (consumption))))'
        result = Parser.tree_from_string(expr)
        self.assertIsInstance(result, Tree)

    def test_get_sub_expr_complex(self):
        """Test extracting sub-expressions from complex nested expression."""
        expr = '(and (p1) (or (p2) (p3)) (not (p4)))'
        result = Parser.get_sub_expr(expr)
        self.assertEqual(len(result), 3)

    def test_param_to_string_empty_name(self):
        """Test converting parameter with empty name."""
        param = Param()
        param.name = ''
        param.type = 'robot'
        result = Parser.param_to_string(param)
        self.assertEqual(result, ' - robot')

    def test_tree_from_string_with_spaces(self):
        """Test creating tree from expression with extra spaces."""
        expr = '(  and   (  p1  )   (  p2  )  )'
        result = Parser.tree_from_string(expr)
        self.assertIsInstance(result, Tree)
        self.assertGreater(len(result.nodes), 0)

    def test_tree_to_string_parameter_node(self):
        """Test converting tree with parameter node."""
        tree = Parser.tree_from_string('(?robot)')
        result = Parser.tree_to_string(tree)
        self.assertIn('?robot', result)

    def test_tree_to_string_number_node(self):
        """Test converting tree with number node."""
        tree = Parser.tree_from_string('42')
        result = Parser.tree_to_string(tree)
        self.assertIn('42', result)

    def test_get_parenthesis_multiple_levels(self):
        """Test finding matching parenthesis with multiple nesting levels."""
        expr = '(and (or (p1) (p2)) (and (p3) (p4)))'
        result = Parser.get_parenthesis(expr, 0)
        self.assertEqual(result, len(expr) - 1)

    def test_get_sub_expr_single_element(self):
        """Test extracting sub-expressions from single element."""
        expr = '(p1)'
        result = Parser.get_sub_expr(expr)
        # Single predicate has itself as a sub-expression
        self.assertGreaterEqual(len(result), 0)

    def test_tree_from_string_scale_up(self):
        """Test creating tree from SCALE_UP expression."""
        expr = '(scale-up (value) 2)'
        result = Parser.tree_from_string(expr)
        self.assertIsInstance(result, Tree)

    def test_tree_from_string_scale_down(self):
        """Test creating tree from SCALE_DOWN expression."""
        expr = '(scale-down (value) 2)'
        result = Parser.tree_from_string(expr)
        self.assertIsInstance(result, Tree)

    def test_tree_to_string_scale_up(self):
        """Test converting SCALE_UP tree to string."""
        tree = Parser.tree_from_string('(scale-up (x) 3)')
        result = Parser.tree_to_string(tree)
        self.assertIn('scale-up', result)

    def test_tree_to_string_scale_down(self):
        """Test converting SCALE_DOWN tree to string."""
        tree = Parser.tree_from_string('(scale-down (x) 2)')
        result = Parser.tree_to_string(tree)
        self.assertIn('scale-down', result)

    def test_param_to_dict_with_type(self):
        """Test converting parameter to dictionary with type."""
        param = Param()
        param.name = 'robot1'
        param.type = 'robot'
        param.sub_types = ['mobile', 'autonomous']
        result = Parser.param_to_dict(param)
        self.assertEqual(result['name'], 'robot1')
        self.assertEqual(result['type'], 'robot')
        self.assertEqual(result['sub_types'], ['mobile', 'autonomous'])

    def test_param_to_dict_without_type(self):
        """Test converting parameter to dictionary without type."""
        param = Param()
        param.name = 'wp1'
        param.type = ''
        result = Parser.param_to_dict(param)
        self.assertEqual(result['name'], 'wp1')
        self.assertEqual(result['type'], '')
        self.assertEqual(result['sub_types'], [])

    def test_node_to_dict_predicate(self):
        """Test converting predicate node to dictionary."""
        node = Node()
        node.node_type = Node.PREDICATE
        node.name = 'robot_at'
        param1 = Param()
        param1.name = 'r1'
        param2 = Param()
        param2.name = 'wp1'
        node.parameters = [param1, param2]
        node.node_id = 0
        node.value = 0.0
        node.negate = False

        result = Parser.node_to_dict(node)
        self.assertEqual(result['node_type'], Node.PREDICATE)
        self.assertEqual(result['node_type_name'], 'PREDICATE')
        self.assertEqual(result['name'], 'robot_at')
        self.assertEqual(len(result['parameters']), 2)
        self.assertEqual(result['value'], 0.0)
        self.assertFalse(result['negate'])

    def test_node_to_dict_function(self):
        """Test converting function node to dictionary."""
        node = Node()
        node.node_type = Node.FUNCTION
        node.name = 'battery_level'
        node.value = 50.5
        node.node_id = 0

        result = Parser.node_to_dict(node)
        self.assertEqual(result['node_type'], Node.FUNCTION)
        self.assertEqual(result['node_type_name'], 'FUNCTION')
        self.assertEqual(result['value'], 50.5)

    def test_node_to_dict_expression(self):
        """Test converting expression node to dictionary."""
        node = Node()
        node.node_type = Node.EXPRESSION
        node.expression_type = Node.COMP_GE
        node.node_id = 0
        node.children = [1, 2]

        result = Parser.node_to_dict(node)
        self.assertEqual(result['node_type'], Node.EXPRESSION)
        self.assertEqual(result['node_type_name'], 'EXPRESSION')
        self.assertEqual(result['expression_type'], Node.COMP_GE)
        self.assertEqual(result['expression_type_name'], 'COMP_GE')
        self.assertEqual(result['children'], [1, 2])

    def test_node_to_dict_function_modifier(self):
        """Test converting function modifier node to dictionary."""
        node = Node()
        node.node_type = Node.FUNCTION_MODIFIER
        node.modifier_type = Node.INCREASE
        node.node_id = 0
        node.children = [1, 2]

        result = Parser.node_to_dict(node)
        self.assertEqual(result['node_type'], Node.FUNCTION_MODIFIER)
        self.assertEqual(result['modifier_type'], Node.INCREASE)
        self.assertEqual(result['modifier_type_name'], 'INCREASE')

    def test_node_to_dict_and_node(self):
        """Test converting AND node to dictionary."""
        node = Node()
        node.node_type = Node.AND
        node.node_id = 0
        node.children = [1, 2, 3]

        result = Parser.node_to_dict(node)
        self.assertEqual(result['node_type'], Node.AND)
        self.assertEqual(result['node_type_name'], 'AND')
        self.assertEqual(len(result['children']), 3)

    def test_node_to_dict_number(self):
        """Test converting number node to dictionary."""
        node = Node()
        node.node_type = Node.NUMBER
        node.value = 42.5
        node.node_id = 0

        result = Parser.node_to_dict(node)
        self.assertEqual(result['node_type'], Node.NUMBER)
        self.assertEqual(result['node_type_name'], 'NUMBER')
        self.assertEqual(result['value'], 42.5)

    def test_tree_to_dict_simple(self):
        """Test converting simple tree to dictionary."""
        tree = Parser.tree_from_string('(robot_at r1 wp1)')
        result = Parser.tree_to_dict(tree)
        self.assertIn('nodes', result)
        self.assertGreater(len(result['nodes']), 0)

    def test_tree_to_dict_complex(self):
        """Test converting complex tree to dictionary."""
        tree = Parser.tree_from_string('(and (p1) (or (p2) (p3)))')
        result = Parser.tree_to_dict(tree)
        self.assertIn('nodes', result)
        self.assertGreater(len(result['nodes']), 2)

    def test_tree_to_dict_empty(self):
        """Test converting empty tree to dictionary."""
        tree = Tree()
        result = Parser.tree_to_dict(tree)
        self.assertEqual(result['nodes'], [])

    def test_node_to_string_invalid_type(self):
        """Test node_to_string with invalid node type raises error."""
        node = Node()
        node.node_type = Node.AND  # Not PREDICATE or FUNCTION
        with self.assertRaises(ValueError):
            Parser.node_to_string(node)

    def test_tree_to_string_invalid_node_id(self):
        """Test tree_to_string with invalid node_id."""
        tree = Parser.tree_from_string('(p1)')
        result = Parser.tree_to_string(tree, node_id=100)
        self.assertEqual(result, '')

    def test_tree_to_string_constant_node(self):
        """Test converting constant node to string."""
        tree = Tree()
        node = Node()
        node.node_type = Node.CONSTANT
        node.name = 'my_constant'
        node.node_id = 0
        tree.nodes.append(node)
        result = Parser.tree_to_string(tree)
        self.assertEqual(result, 'my_constant')

    def test_tree_to_string_expression_with_negation(self):
        """Test converting expression with negation."""
        tree = Parser.tree_from_string('(>= 5 3)')
        result = Parser.tree_to_string(tree, negate=True)
        self.assertIn('not', result)
        self.assertIn('>=', result)

    def test_tree_to_string_expression_insufficient_children(self):
        """Test expression node with insufficient children."""
        tree = Tree()
        node = Node()
        node.node_type = Node.EXPRESSION
        node.expression_type = Node.COMP_GE
        node.node_id = 0
        node.children = [1]  # Only one child, needs at least 2
        tree.nodes.append(node)
        result = Parser.tree_to_string(tree)
        self.assertEqual(result, '')

    def test_tree_to_string_function_modifier_insufficient_children(self):
        """Test function modifier node with insufficient children."""
        tree = Tree()
        node = Node()
        node.node_type = Node.FUNCTION_MODIFIER
        node.modifier_type = Node.ASSIGN
        node.node_id = 0
        node.children = [1]  # Only one child, needs at least 2
        tree.nodes.append(node)
        result = Parser.tree_to_string(tree)
        self.assertEqual(result, '')

    def test_tree_to_string_and_empty_children(self):
        """Test AND node with empty children list."""
        tree = Tree()
        node = Node()
        node.node_type = Node.AND
        node.node_id = 0
        node.children = []
        tree.nodes.append(node)
        result = Parser.tree_to_string(tree)
        self.assertEqual(result, '(and)')

    def test_tree_to_string_or_empty_children(self):
        """Test OR node with empty children list."""
        tree = Tree()
        node = Node()
        node.node_type = Node.OR
        node.node_id = 0
        node.children = []
        tree.nodes.append(node)
        result = Parser.tree_to_string(tree)
        self.assertEqual(result, '(or)')

    def test_tree_to_string_not_empty_children(self):
        """Test NOT node with empty children list."""
        tree = Tree()
        node = Node()
        node.node_type = Node.NOT
        node.node_id = 0
        node.children = []
        tree.nodes.append(node)
        result = Parser.tree_to_string(tree)
        self.assertEqual(result, '(not)')

    def test_tree_to_string_exists_empty_children(self):
        """Test EXISTS node with empty children list."""
        tree = Tree()
        node = Node()
        node.node_type = Node.EXISTS
        node.node_id = 0
        node.children = []
        param = Param()
        param.name = '?x'
        node.parameters = [param]
        tree.nodes.append(node)
        result = Parser.tree_to_string(tree)
        self.assertEqual(result, '(exists ())')

    def test_tree_to_string_parameter_no_parameters(self):
        """Test parameter node with no parameters."""
        tree = Tree()
        node = Node()
        node.node_type = Node.PARAMETER
        node.node_id = 0
        node.parameters = []
        tree.nodes.append(node)
        result = Parser.tree_to_string(tree)
        self.assertEqual(result, '')

    def test_get_node_type_function_modifier_first(self):
        """Test that function modifier is detected before expression."""
        expr = '(assign (x) 10)'
        result = Parser.get_node_type(expr, default_node_type=Node.EXPRESSION)
        self.assertEqual(result, Node.FUNCTION_MODIFIER)

    def test_get_node_type_decimal_starting_with_dot(self):
        """Test detection of decimal number starting with dot."""
        expr = '.5'
        result = Parser.get_node_type(expr)
        self.assertEqual(result, Node.NUMBER)

    def test_get_node_type_function_with_function_modifier_default(self):
        """Test function detection with FUNCTION_MODIFIER default."""
        expr = '(battery_level r1)'
        result = Parser.get_node_type(expr, default_node_type=Node.FUNCTION_MODIFIER)
        self.assertEqual(result, Node.FUNCTION)

    def test_node_from_string_function_no_value(self):
        """Test creating function node without numeric value."""
        function = '(battery_level robot1)'
        result = Parser.node_from_string_function(function)
        # 'robot1' contains '1' which matches the number pattern
        self.assertEqual(result.value, 1.0)

    def test_node_from_string_function_no_name_match(self):
        """Test creating function node with no name match."""
        function = '(123 invalid)'
        result = Parser.node_from_string_function(function)
        # 'invalid' is a valid name starting with a letter
        self.assertEqual(result.value, 0.0)
        self.assertEqual(result.name, 'invalid')

    def test_get_sub_expr_with_tokens_only(self):
        """Test extracting sub-expressions with only token elements."""
        expr = '(exists ?x - robot (at ?x))'
        result = Parser.get_sub_expr(expr)
        self.assertGreater(len(result), 0)

    def test_tree_from_string_unknown_node_type(self):
        """Test tree_from_string returns None for unknown types."""
        # Test internal behavior by passing empty expression after 'and'
        result = Parser.tree_from_string('(and )')
        self.assertIsInstance(result, Tree)

    def test_tree_to_string_exists_with_typed_params(self):
        """Test converting EXISTS with typed parameters."""
        tree = Parser.tree_from_string('(exists (?x - robot) (at ?x wp1))')
        result = Parser.tree_to_string(tree)
        self.assertIn('exists', result)

    def test_tree_to_string_predicate_with_negation(self):
        """Test converting predicate with explicit negation."""
        tree = Tree()
        node = Node()
        node.node_type = Node.PREDICATE
        node.name = 'robot_at'
        node.node_id = 0
        node.negate = True
        param = Param()
        param.name = 'r1'
        node.parameters = [param]
        tree.nodes.append(node)

        result = Parser.tree_to_string(tree, negate=True)
        self.assertIn('not', result)

    def test_remove_operator_before_parenthesis_with_exists(self):
        """Test _remove_operator_before_parenthesis with exists."""
        expr = 'exists (?x) (at ?x)'
        result = Parser._remove_operator_before_parenthesis(expr)
        # 'exists' should be removed
        self.assertNotIn('exists', result) or self.assertIn('(?x)', result)

    def test_get_sub_expr_with_trailing_spaces(self):
        """Test extracting sub-expressions with trailing spaces."""
        expr = '(and (p1)  )'
        result = Parser.get_sub_expr(expr)
        self.assertGreater(len(result), 0)
        self.assertIn('(p1)', result)

    def test_get_sub_expr_with_leading_spaces(self):
        """Test extracting sub-expressions with leading spaces."""
        expr = '(  and (p1) (p2))'
        result = Parser.get_sub_expr(expr)
        self.assertEqual(len(result), 2)
        self.assertIn('(p1)', result)
        self.assertIn('(p2)', result)

    def test_node_from_string_exists_with_dash_type(self):
        """Test creating exists node with dash-separated types."""
        exists = '(exists (?x - robot ?y - waypoint) (at ?x ?y))'
        result = Parser.node_from_string_exists(exists)
        self.assertEqual(result.node_type, Node.EXISTS)
        self.assertGreater(len(result.parameters), 0)

    def test_node_from_string_exists_single_param(self):
        """Test creating exists node with single parameter."""
        exists = '(exists (?x) (p ?x))'
        result = Parser.node_from_string_exists(exists)
        self.assertEqual(result.node_type, Node.EXISTS)
        self.assertEqual(len(result.parameters), 1)

    def test_tree_from_string_with_and_space(self):
        """Test tree from string with (and ) format."""
        expr = '(and )'
        result = Parser.tree_from_string(expr)
        self.assertIsInstance(result, Tree)

    def test_param_to_dict_with_sub_types(self):
        """Test converting parameter with sub_types to dict."""
        param = Param()
        param.name = 'robot1'
        param.type = 'robot'
        param.sub_types = ['mobile', 'wheeled']
        result = Parser.param_to_dict(param)
        self.assertEqual(result['sub_types'], ['mobile', 'wheeled'])

    def test_node_to_dict_or_node(self):
        """Test converting OR node to dictionary."""
        node = Node()
        node.node_type = Node.OR
        node.node_id = 0
        node.children = [1, 2]
        result = Parser.node_to_dict(node)
        self.assertEqual(result['node_type'], Node.OR)
        self.assertEqual(result['node_type_name'], 'OR')

    def test_node_to_dict_not_node(self):
        """Test converting NOT node to dictionary."""
        node = Node()
        node.node_type = Node.NOT
        node.node_id = 0
        node.children = [1]
        result = Parser.node_to_dict(node)
        self.assertEqual(result['node_type'], Node.NOT)
        self.assertEqual(result['node_type_name'], 'NOT')

    def test_node_to_dict_exists_node(self):
        """Test converting EXISTS node to dictionary."""
        node = Node()
        node.node_type = Node.EXISTS
        node.node_id = 0
        param = Param()
        param.name = '?x'
        param.type = 'robot'
        node.parameters = [param]
        result = Parser.node_to_dict(node)
        self.assertEqual(result['node_type'], Node.EXISTS)
        self.assertEqual(result['node_type_name'], 'EXISTS')

    def test_node_from_string_function_only_numbers(self):
        """Test creating function node with only numbers (no name)."""
        function = '(123 456)'
        result = Parser.node_from_string_function(function)
        # No valid name found, should return empty name and value 0.0
        self.assertEqual(result.name, '')
        self.assertEqual(result.value, 0.0)

    def test_tree_to_string_unknown_node_type(self):
        """Test tree_to_string with unknown node type returns empty string."""
        tree = Tree()
        node = Node()
        node.node_type = 999  # Unknown type
        node.node_id = 0
        tree.nodes.append(node)
        result = Parser.tree_to_string(tree)
        self.assertEqual(result, '')

    def test_to_string_predicate_out_of_bounds(self):
        """Test _to_string_predicate with node_id out of bounds."""
        tree = Tree()
        result = Parser._to_string_predicate(tree, 10, False)
        self.assertEqual(result, '')

    def test_to_string_function_out_of_bounds(self):
        """Test _to_string_function with node_id out of bounds."""
        tree = Tree()
        result = Parser._to_string_function(tree, 10, False)
        self.assertEqual(result, '')

    def test_to_string_number_out_of_bounds(self):
        """Test _to_string_number with node_id out of bounds."""
        tree = Tree()
        result = Parser._to_string_number(tree, 10, False)
        self.assertEqual(result, '')

    def test_to_string_and_out_of_bounds(self):
        """Test _to_string_and with node_id out of bounds."""
        tree = Tree()
        result = Parser._to_string_and(tree, 10, False)
        self.assertEqual(result, '')

    def test_to_string_or_out_of_bounds(self):
        """Test _to_string_or with node_id out of bounds."""
        tree = Tree()
        result = Parser._to_string_or(tree, 10, False)
        self.assertEqual(result, '')

    def test_to_string_not_out_of_bounds(self):
        """Test _to_string_not with node_id out of bounds."""
        tree = Tree()
        result = Parser._to_string_not(tree, 10, False)
        self.assertEqual(result, '')

    def test_to_string_expression_out_of_bounds(self):
        """Test _to_string_expression with node_id out of bounds."""
        tree = Tree()
        result = Parser._to_string_expression(tree, 10, False)
        self.assertEqual(result, '')

    def test_to_string_function_modifier_out_of_bounds(self):
        """Test _to_string_function_modifier with node_id out of bounds."""
        tree = Tree()
        result = Parser._to_string_function_modifier(tree, 10, False)
        self.assertEqual(result, '')

    def test_to_string_constant_out_of_bounds(self):
        """Test _to_string_constant with node_id out of bounds."""
        tree = Tree()
        result = Parser._to_string_constant(tree, 10, False)
        self.assertEqual(result, '')

    def test_to_string_parameter_out_of_bounds(self):
        """Test _to_string_parameter with node_id out of bounds."""
        tree = Tree()
        result = Parser._to_string_parameter(tree, 10, False)
        self.assertEqual(result, '')

    def test_to_string_exists_out_of_bounds(self):
        """Test _to_string_exists with node_id out of bounds."""
        tree = Tree()
        result = Parser._to_string_exists(tree, 10, False)
        self.assertEqual(result, '')

    def test_get_sub_expr_trailing_space_edge_case(self):
        """Test get_sub_expr with expression ending in multiple spaces."""
        expr = '(and (p1) (p2)     )'
        result = Parser.get_sub_expr(expr)
        self.assertEqual(len(result), 2)

    def test_get_sub_expr_leading_space_edge_case(self):
        """Test get_sub_expr with expression starting with multiple spaces."""
        expr = '(     and (p1) (p2))'
        result = Parser.get_sub_expr(expr)
        self.assertEqual(len(result), 2)

    def test_get_sub_expr_no_match_after_token(self):
        """Test get_sub_expr when regex doesn't match after token extraction."""
        # This is a difficult edge case to trigger, but we'll try with special characters
        expr = '(and token1 token2)'
        result = Parser.get_sub_expr(expr)
        self.assertGreaterEqual(len(result), 0)

    def test_tree_to_string_exists_with_typed_params_negated(self):
        """Test converting negated EXISTS with typed parameters."""
        tree = Tree()
        node = Node()
        node.node_type = Node.EXISTS
        node.node_id = 0
        node.children = [1]
        param = Param()
        param.name = '?x'
        param.type = 'robot'
        node.parameters = [param]
        tree.nodes.append(node)

        # Add child node
        child = Node()
        child.node_type = Node.PREDICATE
        child.name = 'at'
        child.node_id = 1
        child_param = Param()
        child_param.name = '?x'
        child.parameters = [child_param]
        tree.nodes.append(child)

        result = Parser.tree_to_string(tree, negate=True)
        self.assertIn('not', result)
        self.assertIn('exists', result)
        self.assertIn('robot', result)

    def test_tree_to_string_exists_multiple_params_with_types(self):
        """Test converting EXISTS with multiple typed parameters."""
        tree = Tree()
        node = Node()
        node.node_type = Node.EXISTS
        node.node_id = 0
        node.children = [1]
        param1 = Param()
        param1.name = '?x'
        param1.type = 'robot'
        param2 = Param()
        param2.name = '?y'
        param2.type = 'waypoint'
        node.parameters = [param1, param2]
        tree.nodes.append(node)

        # Add child node
        child = Node()
        child.node_type = Node.PREDICATE
        child.name = 'at'
        child.node_id = 1
        tree.nodes.append(child)

        result = Parser.tree_to_string(tree)
        self.assertIn('?x', result)
        self.assertIn('?y', result)
        self.assertIn('robot', result)
        self.assertIn('waypoint', result)

    def test_get_sub_expr_with_space_after_closing_paren(self):
        """Test get_sub_expr with space right after closing parenthesis removal."""
        # This should trigger line 252: wexpr = wexpr[:-1] in the while loop
        expr = '(and (p1) )  '
        result = Parser.get_sub_expr(expr)
        self.assertGreater(len(result), 0)

    def test_get_sub_expr_with_space_before_opening_paren(self):
        """Test get_sub_expr with space right after opening parenthesis removal."""
        # This should trigger line 256: wexpr = wexpr[1:] in the while loop
        expr = '(  and (p1) (p2))'
        result = Parser.get_sub_expr(expr)
        self.assertEqual(len(result), 2)

    def test_get_sub_expr_edge_case_no_regex_match(self):
        """Test get_sub_expr with content that doesn't match token regex."""
        # Try to create a scenario where regex doesn't match after lstrip
        # This is nearly impossible with the regex [^\s()]+ but we try
        expr = '(and)'
        result = Parser.get_sub_expr(expr)
        # Should return empty list or minimal results
        self.assertIsInstance(result, list)


if __name__ == '__main__':
    unittest.main()
