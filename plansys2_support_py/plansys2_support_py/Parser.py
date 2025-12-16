# Copyright (c) 2025 Alberto J. Tudela Roldán
# Copyright (c) 2025 Grupo Avispa, DTE, Universidad de Málaga
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Parser utilities for PlanSys2 PDDL structures."""

import re
from typing import Any, Dict, List, Optional, Tuple

from plansys2_msgs.msg import Node, Param, Tree


class Parser:
    """
    Parser class for converting between strings and PlanSys2 PDDL structures.

    This class provides utilities to parse and generate strings for PDDL elements
    such as parameters, nodes, and trees, following the same conventions as the
    C++ implementation in plansys2_pddl_parser.
    """

    @staticmethod
    def get_reduced_string(expr: str) -> str:
        """
        Remove newlines, duplicated spaces, tabs and spaces from parenthesis.

        Parameters
        ----------
        expr : str
            The expression to be reduced.

        Returns
        -------
        str
            The reduced expression.

        """
        # Remove newlines and tabs
        result = re.sub(r'[\n\t]+', '', expr)
        # Replace multiple spaces with single space
        result = re.sub(r' +', ' ', result)
        # Remove space after opening parenthesis
        result = re.sub(r'\(\s+', '(', result)
        # Remove space before closing parenthesis
        result = re.sub(r'\s+\)', ')', result)
        return result

    @staticmethod
    def get_node_type(expr: str, default_node_type: int = Node.UNKNOWN) -> int:
        """
        Return node type corresponding to the string input.

        Parameters
        ----------
        expr : str
            The input string.
        default_node_type : int, optional
            Default node type if none is found.

        Returns
        -------
        int
            The node type.

        """
        patterns = [
            (r'\(\s*and[ (]', Node.AND),
            (r'\(\s*or[ (]', Node.OR),
            (r'\(\s*not[ (]', Node.NOT),
            (r'\(\s*exists[ (]', Node.EXISTS),
            (r'^\s*\(?\s*\?\w+', Node.PARAMETER),
        ]

        first_match = None
        first_position = float('inf')

        for pattern, node_type in patterns:
            match = re.search(pattern, expr)
            if match and match.start() < first_position:
                first_position = match.start()
                first_match = node_type

        if first_match is not None:
            return first_match

        # Check for function modifiers
        modifier_type, pos = Parser.get_fun_mod(expr)
        if modifier_type != Node.UNKNOWN and pos < first_position:
            return Node.FUNCTION_MODIFIER

        # Check for expressions
        expr_type, pos = Parser.get_expr(expr)
        if expr_type != Node.UNKNOWN and pos < first_position:
            return Node.EXPRESSION

        # Check for numbers
        if re.match(r'^[+-]?([0-9]+([.][0-9]*)?|[.][0-9]+)$', expr.strip()):
            return Node.NUMBER

        # Check for functions/predicates
        if re.match(r'\(\s*[a-zA-Z]', expr):
            if default_node_type == Node.EXPRESSION or default_node_type == Node.FUNCTION_MODIFIER:
                return Node.FUNCTION
            return default_node_type if default_node_type != Node.UNKNOWN else Node.PREDICATE

        return default_node_type

    @staticmethod
    def get_expr(expr: str) -> Tuple[int, int]:
        """
        Return expression type and start position of an expression in a string.

        Parameters
        ----------
        expr : str
            The input string.

        Returns
        -------
        Tuple[int, int]
            Tuple of (expression_type, start_position).

        """
        patterns = [
            (r'\s*>=', Node.COMP_GE),
            (r'\s*>', Node.COMP_GT),
            (r'\s*<=', Node.COMP_LE),
            (r'\s*<', Node.COMP_LT),
            (r'\s*=', Node.COMP_EQ),
            (r'\s*\*', Node.ARITH_MULT),
            (r'\s*/', Node.ARITH_DIV),
            (r'\s*\+', Node.ARITH_ADD),
            (r'\s*-\s+', Node.ARITH_SUB),
        ]

        first_match = None
        first_position = float('inf')

        for pattern, expr_type in patterns:
            match = re.search(pattern, expr)
            if match and match.start() < first_position:
                first_position = match.start()
                first_match = expr_type

        if first_match is None:
            return (Node.UNKNOWN, -1)

        return (first_match, int(first_position))

    @staticmethod
    def get_fun_mod(expr: str) -> Tuple[int, int]:
        """
        Return function modifier type and start position.

        Parameters
        ----------
        expr : str
            The input string.

        Returns
        -------
        Tuple[int, int]
            Tuple of (modifier_type, start_position).

        """
        patterns = [
            (r'assign', Node.ASSIGN),
            (r'increase', Node.INCREASE),
            (r'decrease', Node.DECREASE),
            (r'scale-up', Node.SCALE_UP),
            (r'scale-down', Node.SCALE_DOWN),
        ]

        first_match = None
        first_position = float('inf')

        for pattern, mod_type in patterns:
            match = re.search(pattern, expr)
            if match and match.start() < first_position:
                first_position = match.start()
                first_match = mod_type

        if first_match is None:
            return (Node.UNKNOWN, -1)

        return (first_match, int(first_position))

    @staticmethod
    def get_parenthesis(expr: str, start: int) -> int:
        """
        Find the matching closing parenthesis.

        Parameters
        ----------
        expr : str
            The expression string.
        start : int
            Starting position of opening parenthesis.

        Returns
        -------
        int
            Position of matching closing parenthesis.

        """
        it = start + 1
        balance = 1

        while it < len(expr):
            if expr[it] == '(':
                balance += 1
            elif expr[it] == ')':
                balance -= 1
                if balance == 0:
                    return it
            it += 1

        return it

    @staticmethod
    def get_sub_expr(expr: str) -> List[str]:
        """
        Extract sub-expressions from a compound expression.

        Parameters
        ----------
        expr : str
            The expression string.

        Returns
        -------
        List[str]
            List of sub-expression strings.

        """
        result = []
        wexpr = expr.strip()

        # Remove outer parentheses
        while wexpr and wexpr[-1] == ' ':
            wexpr = wexpr[:-1]
        if wexpr and wexpr[-1] == ')':
            wexpr = wexpr[:-1]
        while wexpr and wexpr[0] == ' ':
            wexpr = wexpr[1:]
        if wexpr and wexpr[0] == '(':
            wexpr = wexpr[1:]

        # Remove operator before parenthesis
        wexpr = Parser._remove_operator_before_parenthesis(wexpr)

        # Parse inner content
        while wexpr:
            wexpr = wexpr.lstrip()
            if not wexpr:
                break

            if wexpr[0] == '(':
                end = Parser.get_parenthesis(wexpr, 0)
                result.append(wexpr[:end + 1])
                wexpr = wexpr[end + 1:]
            else:
                # Extract token
                match = re.match(r'[^\s()]+', wexpr)
                if match:
                    result.append(match.group())
                    wexpr = wexpr[match.end():]
                else:
                    break

        return result

    @staticmethod
    def _remove_operator_before_parenthesis(expr: str) -> str:
        """
        Remove operator keywords before the first parenthesis.

        Parameters
        ----------
        expr : str
            The expression string.

        Returns
        -------
        str
            Expression with operator removed.

        """
        # Check for 'exists'
        first_paren = expr.find('(')
        exists_pos = expr.find('exists')

        if exists_pos != -1 and (first_paren == -1 or exists_pos < first_paren):
            # Remove 'exists' and surrounding spaces
            expr = re.sub(r'exists\s*', '', expr, count=1)
            return expr

        # Remove other operators
        operator_pattern = (
            r'[\+\-\*/=<>]=?|<=|>=|and|or|not|'
            r'assign|increase|decrease|scale-up|scale-down'
        )
        match = re.search(operator_pattern, expr)
        if match:
            # Remove the operator and any trailing spaces
            expr = expr[:match.start()] + expr[match.end():].lstrip()

        return expr

    @staticmethod
    def param_from_string(name: str, param_type: str = '') -> Param:
        """
        Create a Param from string representation.

        Parameters
        ----------
        name : str
            Parameter name.
        param_type : str, optional
            Parameter type.

        Returns
        -------
        Param
            Param message.

        """
        param = Param()
        param.name = name
        param.type = param_type
        return param

    @staticmethod
    def param_to_string(param: Param) -> str:
        """
        Convert a Param to string representation.

        Parameters
        ----------
        param : Param
            The Param message.

        Returns
        -------
        str
            String representation.

        """
        if param.type:
            return f'{param.name} - {param.type}'
        return param.name

    @staticmethod
    def node_from_string_predicate(predicate: str) -> Node:
        """
        Create a Node from a predicate string.

        Parameters
        ----------
        predicate : str
            The predicate string (e.g., "(at robot1 location1)").

        Returns
        -------
        Node
            Node message with PREDICATE type.

        """
        node = Node()
        node.node_type = Node.PREDICATE

        # Clean up parentheses
        pred = predicate.strip()
        if pred and pred[0] == '(':
            pred = pred[1:]
        if pred and pred[-1] == ')':
            pred = pred[:-1]

        # Split into tokens
        tokens = pred.split()

        if tokens:
            node.name = tokens[0]
            for token in tokens[1:]:
                param = Parser.param_from_string(token)
                node.parameters.append(param)  # type: ignore[union-attr]

        node.value = 0.0
        return node

    @staticmethod
    def node_from_string_function(function: str) -> Node:
        """
        Create a Node from a function string.

        Parameters
        ----------
        function : str
            The function string (e.g., "(battery-level robot1)").

        Returns
        -------
        Node
            Node message with FUNCTION type.

        """
        node = Node()
        node.node_type = Node.FUNCTION

        name_pattern = r'[a-zA-Z][a-zA-Z0-9_\-]*'
        number_pattern = r'[+-]?([0-9]+([.][0-9]*)?|[.][0-9]+)'

        # Extract name
        match = re.search(name_pattern, function)
        if match:
            node.name = match.group()
            temp = function[match.end():]

            # Extract parameters
            for match in re.finditer(name_pattern, temp):
                param = Parser.param_from_string(match.group())
                node.parameters.append(param)  # type: ignore[union-attr]

            # Extract value if present
            match = re.search(number_pattern, temp)
            if match:
                node.value = float(match.group())
            else:
                node.value = 0.0
        else:
            node.value = 0.0

        return node

    @staticmethod
    def node_from_string_exists(exists: str) -> Node:
        """
        Create a Node from an exists expression.

        Parameters
        ----------
        exists : str
            The exists expression string.

        Returns
        -------
        Node
            Node message with EXISTS type.

        """
        node = Node()
        node.node_type = Node.EXISTS

        # Find parameters between first set of parentheses
        start = 0
        end_exists = exists.find(')', start)
        tokens = []

        end = 0
        while end < end_exists:
            space_pos = exists.find(' ', start)
            if space_pos == -1 or space_pos > end_exists:
                tokens.append(exists[start:end_exists])
                break
            tokens.append(exists[start:space_pos])
            start = space_pos + 1
            end = space_pos

        # Parse parameters (skip 'exists' keyword)
        if len(tokens) > 1:
            # Remove parentheses from first and last tokens
            tokens[-1] = tokens[-1].rstrip(')')
            tokens[1] = tokens[1].lstrip('(')

            for i in range(1, len(tokens)):
                # Parse "name - type" format
                parts = tokens[i].split('-')
                if len(parts) == 2:
                    param = Parser.param_from_string(parts[0].strip(), parts[1].strip())
                else:
                    param = Parser.param_from_string(tokens[i].strip())
                node.parameters.append(param)  # type: ignore[union-attr]

        node.value = 0.0
        return node

    @staticmethod
    def tree_from_string(
            expr: str, negate: bool = False,
            parent: int = Node.UNKNOWN) -> Tree:
        """
        Create a Tree from a string expression.

        Parameters
        ----------
        expr : str
            The expression string.
        negate : bool, optional
            Whether to negate the expression.
        parent : int, optional
            Parent node type for context.

        Returns
        -------
        Tree
            Tree message.

        """
        tree = Tree()
        Parser._from_string_recursive(tree, expr, negate, parent)
        return tree

    @staticmethod
    def _from_string_recursive(tree: Tree, expr: str, negate: bool = False,
                               parent: int = Node.UNKNOWN) -> Optional[Node]:
        """
        Recursively parse expression and build tree.

        Parameters
        ----------
        tree : Tree
            The tree being constructed.
        expr : str
            The expression string.
        negate : bool, optional
            Whether to negate the expression.
        parent : int, optional
            Parent node type for context.

        Returns
        -------
        Optional[Node]
            The created node.

        """
        wexpr = Parser.get_reduced_string(expr)

        if wexpr in ['(and)', '(and )']:
            return None

        # Determine default node type based on parent
        default_node_type: int = Node.UNKNOWN
        if parent in [Node.AND, Node.OR, Node.NOT]:
            default_node_type = Node.PREDICATE
        elif parent in [Node.EXPRESSION, Node.FUNCTION_MODIFIER]:
            default_node_type = Node.FUNCTION

        node_type = Parser.get_node_type(wexpr, default_node_type)

        if node_type == Node.AND:
            node = Node()
            node.node_type = node_type
            node.node_id = len(tree.nodes)
            node.negate = negate
            tree.nodes.append(node)  # type: ignore[union-attr]

            subexprs = Parser.get_sub_expr(wexpr)
            for subexpr in subexprs:
                child = Parser._from_string_recursive(tree, subexpr, negate, node_type)
                if child:
                    tree.nodes[node.node_id].children.append(child.node_id)  # type: ignore[index]

            return node

        elif node_type == Node.OR:
            node = Node()
            node.node_type = node_type
            node.node_id = len(tree.nodes)
            node.negate = negate
            tree.nodes.append(node)  # type: ignore[union-attr]

            subexprs = Parser.get_sub_expr(wexpr)
            for subexpr in subexprs:
                child = Parser._from_string_recursive(tree, subexpr, negate, node_type)
                if child:
                    tree.nodes[node.node_id].children.append(child.node_id)  # type: ignore[index]

            return node

        elif node_type == Node.NOT:
            node = Node()
            node.node_type = node_type
            node.node_id = len(tree.nodes)
            node.negate = negate
            tree.nodes.append(node)  # type: ignore[union-attr]

            subexprs = Parser.get_sub_expr(wexpr)
            if subexprs:
                child = Parser._from_string_recursive(tree, subexprs[0], not negate, node_type)
                if child:
                    tree.nodes[node.node_id].children.append(child.node_id)  # type: ignore[index]

            return node

        elif node_type == Node.PREDICATE:
            node = Parser.node_from_string_predicate(wexpr)
            node.node_id = len(tree.nodes)
            node.negate = negate
            tree.nodes.append(node)  # type: ignore[union-attr]
            return node

        elif node_type == Node.FUNCTION:
            node = Parser.node_from_string_function(wexpr)
            node.node_id = len(tree.nodes)
            node.negate = negate
            tree.nodes.append(node)  # type: ignore[union-attr]
            return node

        elif node_type == Node.EXPRESSION:
            node = Node()
            node.node_type = node_type
            expr_type, _ = Parser.get_expr(wexpr)
            node.expression_type = expr_type
            node.node_id = len(tree.nodes)
            node.negate = negate
            tree.nodes.append(node)  # type: ignore[union-attr]

            subexprs = Parser.get_sub_expr(wexpr)
            for subexpr in subexprs:
                child = Parser._from_string_recursive(tree, subexpr, negate, node_type)
                if child:
                    tree.nodes[node.node_id].children.append(child.node_id)  # type: ignore[index]

            return node

        elif node_type == Node.FUNCTION_MODIFIER:
            node = Node()
            node.node_type = node_type
            mod_type, _ = Parser.get_fun_mod(wexpr)
            node.modifier_type = mod_type
            node.node_id = len(tree.nodes)
            node.negate = negate
            tree.nodes.append(node)  # type: ignore[union-attr]

            subexprs = Parser.get_sub_expr(wexpr)
            for subexpr in subexprs:
                child = Parser._from_string_recursive(tree, subexpr, negate, node_type)
                if child:
                    tree.nodes[node.node_id].children.append(child.node_id)  # type: ignore[index]

            return node

        elif node_type == Node.NUMBER:
            node = Node()
            node.node_type = node_type
            node.node_id = len(tree.nodes)
            node.value = float(wexpr)
            node.negate = negate
            tree.nodes.append(node)  # type: ignore[union-attr]
            return node

        elif node_type == Node.EXISTS:
            node = Parser.node_from_string_exists(wexpr)
            node.node_id = len(tree.nodes)
            node.negate = negate
            tree.nodes.append(node)  # type: ignore[union-attr]

            subexprs = Parser.get_sub_expr(wexpr)
            for subexpr in subexprs:
                child = Parser._from_string_recursive(tree, subexpr, negate, node_type)
                if child:
                    tree.nodes[node.node_id].children.append(child.node_id)  # type: ignore[index]

            return node

        elif node_type == Node.PARAMETER:
            node = Node()
            node.node_type = node_type
            node.node_id = len(tree.nodes)
            node.name = wexpr
            node.negate = negate
            node.parameters.append(Parser.param_from_string(wexpr))  # type: ignore[union-attr]
            tree.nodes.append(node)  # type: ignore[union-attr]
            return node

        return None

    @staticmethod
    def node_to_string(node: Node) -> str:
        """
        Convert a Node to string representation.

        Parameters
        ----------
        node : Node
            The Node message.

        Returns
        -------
        str
            String representation.

        """
        if node.node_type not in [Node.PREDICATE, Node.FUNCTION]:
            raise ValueError(
                'Only PREDICATE and FUNCTION nodes can be converted '
                'without tree context'
            )

        tree = Tree()
        tree.nodes.append(node)  # type: ignore[union-attr]
        return Parser.tree_to_string(tree)

    @staticmethod
    def tree_to_string(tree: Tree, node_id: int = 0, negate: bool = False) -> str:
        """
        Convert a Tree to string representation.

        Parameters
        ----------
        tree : Tree
            The Tree message.
        node_id : int, optional
            Starting node ID (default: 0).
        negate : bool, optional
            Whether to negate the expression.

        Returns
        -------
        str
            String representation.

        """
        if node_id >= len(tree.nodes):
            return ''

        node = tree.nodes[node_id]  # type: ignore[index]

        if node.node_type == Node.PREDICATE:
            return Parser._to_string_predicate(tree, node_id, negate)
        elif node.node_type == Node.FUNCTION:
            return Parser._to_string_function(tree, node_id, negate)
        elif node.node_type == Node.NUMBER:
            return Parser._to_string_number(tree, node_id, negate)
        elif node.node_type == Node.AND:
            return Parser._to_string_and(tree, node_id, negate)
        elif node.node_type == Node.OR:
            return Parser._to_string_or(tree, node_id, negate)
        elif node.node_type == Node.NOT:
            return Parser._to_string_not(tree, node_id, negate)
        elif node.node_type == Node.EXPRESSION:
            return Parser._to_string_expression(tree, node_id, negate)
        elif node.node_type == Node.FUNCTION_MODIFIER:
            return Parser._to_string_function_modifier(tree, node_id, negate)
        elif node.node_type == Node.CONSTANT:
            return Parser._to_string_constant(tree, node_id, negate)
        elif node.node_type == Node.PARAMETER:
            return Parser._to_string_parameter(tree, node_id, negate)
        elif node.node_type == Node.EXISTS:
            return Parser._to_string_exists(tree, node_id, negate)

        return ''

    @staticmethod
    def _to_string_predicate(tree: Tree, node_id: int, negate: bool) -> str:
        """
        Convert predicate node to string.

        Parameters
        ----------
        tree : Tree
            The tree structure.
        node_id : int
            Node identifier.
        negate : bool
            Whether to negate.

        Returns
        -------
        str
            String representation.

        """
        if node_id >= len(tree.nodes):
            return ''

        node = tree.nodes[node_id]  # type: ignore[index]
        result = '(not (' if negate else '('
        result += node.name

        for param in node.parameters:
            result += ' ' + param.name

        result += '))' if negate else ')'
        return result

    @staticmethod
    def _to_string_function(tree: Tree, node_id: int, negate: bool) -> str:
        """
        Convert function node to string.

        Parameters
        ----------
        tree : Tree
            The tree structure.
        node_id : int
            Node identifier.
        negate : bool
            Whether to negate.

        Returns
        -------
        str
            String representation.

        """
        if node_id >= len(tree.nodes):
            return ''

        node = tree.nodes[node_id]  # type: ignore[index]
        result = '(' + node.name

        for param in node.parameters:
            result += ' ' + param.name

        result += ')'
        return result

    @staticmethod
    def _to_string_number(tree: Tree, node_id: int, negate: bool) -> str:
        """
        Convert number node to string.

        Parameters
        ----------
        tree : Tree
            The tree structure.
        node_id : int
            Node identifier.
        negate : bool
            Whether to negate.

        Returns
        -------
        str
            String representation.

        """
        if node_id >= len(tree.nodes):
            return ''

        return str(tree.nodes[node_id].value)  # type: ignore[index]

    @staticmethod
    def _to_string_and(tree: Tree, node_id: int, negate: bool) -> str:
        """
        Convert AND node to string.

        Parameters
        ----------
        tree : Tree
            The tree structure.
        node_id : int
            Node identifier.
        negate : bool
            Whether to negate.

        Returns
        -------
        str
            String representation.

        """
        if node_id >= len(tree.nodes):
            return ''

        node = tree.nodes[node_id]  # type: ignore[index]
        if not node.children:
            return '(and)'

        result = '(or ' if negate else '(and '

        for child_id in node.children:
            result += Parser.tree_to_string(tree, child_id, negate) + ' '

        result += ')'
        return result

    @staticmethod
    def _to_string_or(tree: Tree, node_id: int, negate: bool) -> str:
        """
        Convert OR node to string.

        Parameters
        ----------
        tree : Tree
            The tree structure.
        node_id : int
            Node identifier.
        negate : bool
            Whether to negate.

        Returns
        -------
        str
            String representation.

        """
        if node_id >= len(tree.nodes):
            return ''

        node = tree.nodes[node_id]  # type: ignore[index]
        if not node.children:
            return '(or)'

        result = '(and ' if negate else '(or '

        for child_id in node.children:
            result += Parser.tree_to_string(tree, child_id, negate) + ' '

        result += ')'
        return result

    @staticmethod
    def _to_string_not(tree: Tree, node_id: int, negate: bool) -> str:
        """
        Convert NOT node to string.

        Parameters
        ----------
        tree : Tree
            The tree structure.
        node_id : int
            Node identifier.
        negate : bool
            Whether to negate.

        Returns
        -------
        str
            String representation.

        """
        if node_id >= len(tree.nodes):
            return ''

        node = tree.nodes[node_id]  # type: ignore[index]
        if not node.children:
            return '(not)'

        return Parser.tree_to_string(tree, node.children[0], not negate)

    @staticmethod
    def _to_string_expression(tree: Tree, node_id: int, negate: bool) -> str:
        """
        Convert expression node to string.

        Parameters
        ----------
        tree : Tree
            The tree structure.
        node_id : int
            Node identifier.
        negate : bool
            Whether to negate.

        Returns
        -------
        str
            String representation.

        """
        if node_id >= len(tree.nodes):
            return ''

        node = tree.nodes[node_id]  # type: ignore[index]
        if len(node.children) < 2:
            return ''

        result = '(not ' if negate else ''

        expr_map = {
            Node.COMP_GE: '>=',
            Node.COMP_GT: '>',
            Node.COMP_LE: '<=',
            Node.COMP_LT: '<',
            Node.COMP_EQ: '=',
            Node.ARITH_MULT: '*',
            Node.ARITH_DIV: '/',
            Node.ARITH_ADD: '+',
            Node.ARITH_SUB: '-',
        }

        operator = expr_map.get(node.expression_type, 'UNKNOWN')
        result += f'({operator} '

        for child_id in node.children:
            result += Parser.tree_to_string(tree, child_id, False) + ' '

        result += ')'
        if negate:
            result += ')'

        return result

    @staticmethod
    def _to_string_function_modifier(tree: Tree, node_id: int, negate: bool) -> str:
        """
        Convert function modifier node to string.

        Parameters
        ----------
        tree : Tree
            The tree structure.
        node_id : int
            Node identifier.
        negate : bool
            Whether to negate.

        Returns
        -------
        str
            String representation.

        """
        if node_id >= len(tree.nodes):
            return ''

        node = tree.nodes[node_id]  # type: ignore[index]
        if len(node.children) < 2:
            return ''

        mod_map = {
            Node.ASSIGN: 'assign',
            Node.INCREASE: 'increase',
            Node.DECREASE: 'decrease',
            Node.SCALE_UP: 'scale-up',
            Node.SCALE_DOWN: 'scale-down',
        }

        modifier = mod_map.get(node.modifier_type, '')
        result = f'({modifier} '

        for child_id in node.children:
            result += Parser.tree_to_string(tree, child_id, False) + ' '

        result += ')'
        return result

    @staticmethod
    def _to_string_constant(tree: Tree, node_id: int, negate: bool) -> str:
        """
        Convert constant node to string.

        Parameters
        ----------
        tree : Tree
            The tree structure.
        node_id : int
            Node identifier.
        negate : bool
            Whether to negate.

        Returns
        -------
        str
            String representation.

        """
        if node_id >= len(tree.nodes):
            return ''

        return tree.nodes[node_id].name  # type: ignore[index]

    @staticmethod
    def _to_string_parameter(tree: Tree, node_id: int, negate: bool) -> str:
        """
        Convert parameter node to string.

        Parameters
        ----------
        tree : Tree
            The tree structure.
        node_id : int
            Node identifier.
        negate : bool
            Whether to negate.

        Returns
        -------
        str
            String representation.

        """
        if node_id >= len(tree.nodes):
            return ''

        node = tree.nodes[node_id]  # type: ignore[index]
        if node.parameters:
            return node.parameters[0].name  # type: ignore[index]
        return ''

    @staticmethod
    def _to_string_exists(tree: Tree, node_id: int, negate: bool) -> str:
        """
        Convert exists node to string.

        Parameters
        ----------
        tree : Tree
            The tree structure.
        node_id : int
            Node identifier.
        negate : bool
            Whether to negate.

        Returns
        -------
        str
            String representation.

        """
        if node_id >= len(tree.nodes):
            return ''

        node = tree.nodes[node_id]  # type: ignore[index]
        if not node.children:
            return '(exists ())'

        result = '(not ' if negate else ''
        result += '(exists ('

        first = True
        for param in node.parameters:
            if not first:
                result += ' '
            result += param.name
            if param.type:
                result += f' - {param.type}'
            first = False

        result += ') '

        for child_id in node.children:
            result += Parser.tree_to_string(tree, child_id, False) + ' '

        result += ')'
        if negate:
            result += ')'

        return result

    @staticmethod
    def param_to_dict(param: Param) -> Dict[str, Any]:
        """
        Convert a plansys2_msgs.msg.Param to a dictionary.

        Parameters
        ----------
        param : plansys2_msgs.msg.Param
            The Param message to convert.

        Returns
        -------
        Dict[str, Any]
            Dictionary representation of the Param with keys:
            - name: str - Parameter name
            - type: str - Parameter type
            - sub_types: List[str] - List of sub-types (for complex types)

        """
        return {
            'name': param.name,
            'type': param.type,
            'sub_types': list(param.sub_types) if param.sub_types else []
        }

    @staticmethod
    def node_to_dict(node: Node) -> Dict[str, Any]:
        """
        Convert a plansys2_msgs.msg.Node to a dictionary.

        The Node message represents elements in PDDL expression trees,
        including logical operators, predicates, functions, and values.

        Parameters
        ----------
        node : plansys2_msgs.msg.Node
            The Node message to convert.

        Returns
        -------
        Dict[str, Any]
            Dictionary representation of the Node with keys:
            - node_type: int - Type of node (AND, OR, NOT, PREDICATE, etc.)
            - node_type_name: str - Human-readable name of the node type
            - expression_type: int - Type of expression if applicable
            - expression_type_name: str - Human-readable expression type name
            - modifier_type: int - Type of function modifier if applicable
            - modifier_type_name: str - Human-readable modifier type name
            - node_id: int - Unique identifier for this node
            - children: List[int] - List of child node IDs
            - name: str - Node name (for predicates, functions, etc.)
            - parameters: List[Dict] - List of parameters (converted to dicts)
            - value: float - Numeric value (for NUMBER nodes)
            - negate: bool - Whether the node is negated

        """
        # Node type constants and names mapping
        node_type_names = {
            0: 'UNKNOWN', 1: 'AND', 2: 'OR', 3: 'NOT', 4: 'ACTION',
            5: 'PREDICATE', 6: 'FUNCTION', 7: 'EXPRESSION',
            8: 'FUNCTION_MODIFIER', 9: 'NUMBER', 10: 'CONSTANT',
            11: 'PARAMETER', 12: 'EXISTS'
        }

        # Expression type constants and names mapping
        expression_type_names = {
            13: 'COMP_EQ', 14: 'COMP_GE', 15: 'COMP_GT',
            16: 'COMP_LE', 17: 'COMP_LT', 18: 'ARITH_MULT',
            19: 'ARITH_DIV', 20: 'ARITH_ADD', 21: 'ARITH_SUB'
        }

        # Function modifier type constants and names mapping
        modifier_type_names = {
            22: 'ASSIGN', 23: 'INCREASE', 24: 'DECREASE',
            25: 'SCALE_UP', 26: 'SCALE_DOWN'
        }

        return {
            'node_type': node.node_type,
            'node_type_name': node_type_names.get(node.node_type, 'UNKNOWN'),
            'expression_type': node.expression_type,
            'expression_type_name': expression_type_names.get(
                node.expression_type, 'NONE'
            ),
            'modifier_type': node.modifier_type,
            'modifier_type_name': modifier_type_names.get(
                node.modifier_type, 'NONE'
            ),
            'node_id': node.node_id,
            'children': list(node.children) if node.children else [],
            'name': node.name,
            'parameters': [Parser.param_to_dict(p) for p in node.parameters]
            if node.parameters else [],
            'value': node.value,
            'negate': node.negate
        }

    @staticmethod
    def tree_to_dict(tree: Tree) -> Dict[str, Any]:
        """
        Convert a plansys2_msgs.msg.Tree to a dictionary.

        The Tree message represents a PDDL construct as a tree structure
        composed of Node elements.

        Parameters
        ----------
        tree : plansys2_msgs.msg.Tree
            The Tree message to convert.

        Returns
        -------
        Dict[str, Any]
            Dictionary representation of the Tree with keys:
            - nodes: List[Dict] - List of nodes (converted to dicts)

        """
        nodes = [Parser.node_to_dict(n) for n in tree.nodes] if tree.nodes else []
        return {
            'nodes': nodes
        }
