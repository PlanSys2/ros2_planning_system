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

from decimal import Decimal
import math
import re
from typing import Any, Dict, List, Optional, Tuple

from plansys2_msgs.msg import Metric, Node, Param, Tree

# The same limit as the C++ parser
MAX_NESTING_DEPTH = 256

_BLANKS = ' \t\n\r\f\v'
_BLANK_SPLIT = re.compile('[ \t\n\r\f\v]+')
_NUMBER = re.compile(r'-?([0-9]+\.?[0-9]*|\.[0-9]+)([eE][+-]?[0-9]+)?')
_NAME = re.compile(r'[a-zA-Z][a-zA-Z0-9_-]*')
_ASCII_LOWER = str.maketrans('ABCDEFGHIJKLMNOPQRSTUVWXYZ', 'abcdefghijklmnopqrstuvwxyz')

_EXPRESSIONS = {
    '>=': Node.COMP_GE, '>': Node.COMP_GT, '<=': Node.COMP_LE, '<': Node.COMP_LT,
    '=': Node.COMP_EQ, '*': Node.ARITH_MULT, '/': Node.ARITH_DIV, '+': Node.ARITH_ADD,
    '-': Node.ARITH_SUB,
}
_MODIFIERS = {
    'assign': Node.ASSIGN, 'increase': Node.INCREASE, 'decrease': Node.DECREASE,
    'scale-up': Node.SCALE_UP, 'scale-down': Node.SCALE_DOWN,
}
_LOGIC = {
    'and': Node.AND, 'or': Node.OR, 'not': Node.NOT, 'exists': Node.EXISTS,
    'forall': Node.FORALL, 'imply': Node.IMPLY, 'when': Node.WHEN,
}
# PDDL3 modal constraints: how many times and conditions each one takes
_CONSTRAINTS = {
    'always': (0, 1), 'sometime': (0, 1), 'at-most-once': (0, 1), 'at end': (0, 1),
    'within': (1, 1), 'hold-after': (1, 1), 'sometime-after': (0, 2),
    'sometime-before': (0, 2), 'always-within': (1, 2), 'hold-during': (2, 1),
}
_COMPARISONS = (Node.COMP_GE, Node.COMP_GT, Node.COMP_LE, Node.COMP_LT, Node.COMP_EQ)


def _skip_blanks(text: str, pos: int) -> int:
    while pos < len(text) and text[pos] in _BLANKS:
        pos += 1
    return pos


def _token_at(text: str, pos: int) -> str:
    # Up to a blank or a parenthesis; a symbolic operator ends where its symbols end, as in
    # '(=?a ?b)', and '-' or '+' before a digit is a sign
    if pos < len(text) and text[pos] in '<>=*/+-':
        sign = (text[pos] in '+-' and pos + 1 < len(text) and
                (text[pos + 1] in '0123456789.'))
        if not sign:
            two = text[pos] in '<>' and pos + 1 < len(text) and text[pos + 1] == '='
            return text[pos:pos + (2 if two else 1)]
    end = pos
    while end < len(text) and text[end] not in _BLANKS and text[end] not in '()':
        end += 1
    return text[pos:end]


def _head_token(text: str) -> Tuple[str, int]:
    # The first token after the opening '(', or the first token without one
    pos = _skip_blanks(text, 0)
    if pos < len(text) and text[pos] == '(':
        pos = _skip_blanks(text, pos + 1)
    return _token_at(text, pos), pos


def _split_blanks(text: str) -> List[str]:
    return [token for token in _BLANK_SPLIT.split(text) if token]


class Parser:
    """
    Parser class for converting between strings and PlanSys2 PDDL structures.

    This class provides utilities to parse and generate strings for PDDL elements
    such as parameters, nodes, and trees, with the same results as the C++
    implementation in plansys2_pddl_parser.
    """

    @staticmethod
    def get_max_nesting(expr: str) -> int:
        """
        Return the deepest nesting of parentheses, skipping ';' comments.

        Parameters
        ----------
        expr : str
            The expression.

        Returns
        -------
        int
            The deepest nesting, or -1 if the parentheses are unbalanced.

        """
        depth = 0
        max_depth = 0
        in_comment = False
        for char in expr:
            if in_comment:
                in_comment = char != '\n'
            elif char == ';':
                in_comment = True
            elif char == '(':
                depth += 1
                max_depth = max(max_depth, depth)
            elif char == ')':
                depth -= 1
                if depth < 0:
                    return -1
        return max_depth if depth == 0 else -1

    @staticmethod
    def validate_tree(tree: Tree) -> Tuple[bool, str]:
        """
        Check that every node has the children its type needs, and that it is a tree.

        Parameters
        ----------
        tree : Tree
            The tree to check.

        Returns
        -------
        Tuple[bool, str]
            Whether the tree is valid, and the error if it is not.

        """
        if not tree.nodes:
            return True, ''
        visited = [False] * len(tree.nodes)
        visited[0] = True
        pending = [(0, 1)]
        while pending:
            node_id, depth = pending.pop()
            node = tree.nodes[node_id]
            if depth > MAX_NESTING_DEPTH:
                return False, f'tree deeper than {MAX_NESTING_DEPTH} levels'
            children = len(node.children)
            if node.node_type in (Node.AND, Node.OR):
                limits = (0, children)
            elif node.node_type == Node.EXISTS:
                limits = (1, children)
            elif node.node_type in (Node.NOT, Node.FORALL, Node.PREFERENCE):
                limits = (1, 1)
            elif node.node_type == Node.CONSTRAINT:
                if node.name not in _CONSTRAINTS:
                    return False, f'node {node_id} is no constraint: {node.name}'
                limits = (sum(_CONSTRAINTS[node.name]),) * 2
            elif node.node_type in (Node.IMPLY, Node.WHEN, Node.EXPRESSION):
                limits = (2, 2)
            elif node.node_type == Node.FUNCTION_MODIFIER:
                # Trees built before (total-cost) was a child have only the value
                limits = (1, 2)
            elif node.node_type in (Node.PREDICATE, Node.FUNCTION, Node.NUMBER,
                                    Node.CONSTANT, Node.PARAMETER):
                limits = (0, 0)
            else:
                return False, f'node {node_id} has unknown type {node.node_type}'
            if children < limits[0] or children > limits[1]:
                return False, f'node {node_id} has {children} children'
            for child in node.children:
                if child >= len(tree.nodes):
                    return False, f'node {node_id} has child {child} out of range'
                if visited[child]:
                    return False, f'node {child} is reached twice (cycle or shared node)'
                visited[child] = True
                pending.append((child, depth + 1))
        return True, ''

    @staticmethod
    def is_valid_name(name: str) -> bool:
        """
        Return whether a name is a valid PDDL name: a letter, then letters, digits, '-' or '_'.

        Parameters
        ----------
        name : str
            The name.

        Returns
        -------
        bool
            Whether it is valid.

        """
        return _NAME.fullmatch(name) is not None

    @staticmethod
    def parse_number(text: str) -> Optional[float]:
        """
        Read a PDDL number, whatever the locale, with an optional exponent.

        Parameters
        ----------
        text : str
            The text, with optional blanks around it.

        Returns
        -------
        Optional[float]
            The number, or None if the text is not a finite number.

        """
        text = text.strip(_BLANKS)
        if text.startswith('+'):
            text = text[1:]
            if text.startswith('-'):
                return None
        if _NUMBER.fullmatch(text) is None:
            return None
        value = float(text)
        if not math.isfinite(value):
            return None
        # Too small to be represented, as in 1e-400
        if value == 0.0 and any(c in '123456789' for c in text.split('e')[0].split('E')[0]):
            return None
        return value

    @staticmethod
    def number_to_string(value: float, decimals: int = 6) -> str:
        """
        Write a number in fixed notation, as the C++ numberToString.

        Parameters
        ----------
        value : float
            The number.
        decimals : int, optional
            The decimals to write when they keep the exact value; with fewer than 0, or when
            they would change it, the shortest form that reads back the same is written.

        Returns
        -------
        str
            The number.

        """
        if not math.isfinite(value):
            return f'{value:f}'
        if decimals >= 0:
            text = f'{value:.{decimals}f}'
            if float(text) == value:
                return text
        return format(Decimal(repr(value)).normalize(), 'f')

    @staticmethod
    def get_reduced_string(expr: str) -> str:
        """
        Collapse blanks: one space between tokens, none after '(' or before ')' or at the ends.

        Parameters
        ----------
        expr : str
            The expression to be reduced.

        Returns
        -------
        str
            The reduced expression.

        """
        result = []
        blank = False
        for char in expr:
            if char in _BLANKS:
                blank = True
                continue
            if blank and result and result[-1] != '(' and char != ')':
                result.append(' ')
            blank = False
            result.append(char)
        return ''.join(result)

    @staticmethod
    def get_node_type(expr: str, default_node_type: int = Node.UNKNOWN) -> int:
        """
        Return the node type of an expression, given by its operator.

        A name that contains an operator, as task_assigned, is not that operator.

        Parameters
        ----------
        expr : str
            The input string.
        default_node_type : int, optional
            The type of a name or of an expression without an operator.

        Returns
        -------
        int
            The node type.

        """
        start = _skip_blanks(expr, 0)
        if start == len(expr):
            return default_node_type
        if expr[start] != '(':
            token = _token_at(expr, start)
            # A variable, and #t: the time a continuous effect has acted
            if token.startswith('?') or token == '#t':
                return Node.PARAMETER
            if Parser.parse_number(token) is not None:
                return Node.NUMBER
            return default_node_type
        head, _ = _head_token(expr)
        if not head:
            return default_node_type
        if head.startswith('?'):
            return Node.PARAMETER
        modal = Parser.split_modal(expr)
        if modal is not None:
            return Node.PREFERENCE if modal[0] == 'preference' else Node.CONSTRAINT
        if head in _LOGIC:
            return _LOGIC[head]
        if head in _MODIFIERS:
            return Node.FUNCTION_MODIFIER
        if head in _EXPRESSIONS:
            return Node.EXPRESSION
        return default_node_type

    @staticmethod
    def split_modal(expr: str) -> Optional[Tuple[str, List[str]]]:
        """
        Split a preference or a modal constraint into its operator and arguments.

        Parameters
        ----------
        expr : str
            The input string.

        Returns
        -------
        Optional[Tuple[str, List[str]]]
            The operator ('at end' in full) and its arguments; None for anything else, such
            as a predicate called always.

        """
        head, _ = _head_token(expr)
        if head not in ('preference', 'at') and head not in _CONSTRAINTS:
            return None
        rest = Parser.get_reduced_string(expr)
        if len(rest) < 2 or rest[0] != '(' or rest[-1] != ')':
            return None
        rest = rest[1:-1]
        rest = rest[rest.find(head) + len(head):]
        args: List[str] = []
        i = 0
        while i < len(rest):
            if rest[i] in _BLANKS:
                i += 1
            elif rest[i] == '(':
                last = Parser.get_parenthesis(rest, i)
                if last >= len(rest):
                    return None  # Unbalanced
                args.append(rest[i:last + 1])
                i = last + 1
            else:
                token = _token_at(rest, i)
                if not token:
                    return None
                args.append(token)
                i += len(token)

        def is_cond(arg: str) -> bool:
            return arg.startswith('(')

        if head == 'preference':
            # (preference [name] C)
            if (not args or len(args) > 2 or not is_cond(args[-1]) or
                    (len(args) == 2 and is_cond(args[0]))):
                return None
            return head, args
        if head == 'at':
            if len(args) != 2 or args[0] != 'end' or not is_cond(args[1]):
                return None
            return 'at end', args[1:]
        times, conds = _CONSTRAINTS[head]
        if len(args) != times + conds:
            return None
        for index, arg in enumerate(args):
            if (Parser.parse_number(arg) is None) if index < times else not is_cond(arg):
                return None
        return head, args

    @staticmethod
    def get_expr(expr: str) -> Tuple[int, int]:
        """
        Return the expression type of the operator of an expression and its position.

        Parameters
        ----------
        expr : str
            The input string.

        Returns
        -------
        Tuple[int, int]
            The expression type and the position of the operator; (UNKNOWN, -1) if the
            operator is not a comparison or an arithmetic operation.

        """
        head, position = _head_token(expr)
        if head not in _EXPRESSIONS:
            return (Node.UNKNOWN, -1)
        return (_EXPRESSIONS[head], position)

    @staticmethod
    def get_fun_mod(expr: str) -> Tuple[int, int]:
        """
        Return the modifier type of the operator of an expression and its position.

        Parameters
        ----------
        expr : str
            The input string.

        Returns
        -------
        Tuple[int, int]
            The modifier type and the position of the operator; (UNKNOWN, -1) if the
            operator is not a function modifier.

        """
        head, position = _head_token(expr)
        if head not in _MODIFIERS:
            return (Node.UNKNOWN, -1)
        return (_MODIFIERS[head], position)

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
            Position of matching closing parenthesis, or the length of the expression.

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
        Return the operands of an expression in parentheses.

        Parameters
        ----------
        expr : str
            The expression string.

        Returns
        -------
        List[str]
            The operands; empty if the expression is not in parentheses.

        """
        result: List[str] = []
        wexpr = expr.strip(_BLANKS)
        if len(wexpr) < 2 or wexpr[0] != '(' or wexpr[-1] != ')':
            return result
        wexpr = Parser._remove_operator_before_parenthesis(wexpr[1:-1])
        while wexpr:
            first = wexpr.find('(')
            if first == -1:
                result.extend(_split_blanks(wexpr))
                break
            result.extend(_split_blanks(wexpr[:first]))
            last = Parser.get_parenthesis(wexpr, first)
            result.append(wexpr[first:last + 1])
            wexpr = wexpr[last + 1:]
        return result

    @staticmethod
    def _remove_operator_before_parenthesis(expr: str) -> str:
        """
        Remove the operator that starts the text; a name that contains one is left alone.

        Parameters
        ----------
        expr : str
            The text inside the parentheses of an expression.

        Returns
        -------
        str
            The text without the operator, and without the variable list of exists or forall.

        """
        start = _skip_blanks(expr, 0)
        op = _token_at(expr, start)
        end = start + len(op)
        if op in ('exists', 'forall'):
            open_list = expr.find('(', end)
            close_list = -1 if open_list == -1 else expr.find(')', open_list)
            if close_list == -1:
                raise ValueError(f"Malformed '{op}' expression: missing variable list")
            end = close_list + 1
        elif op not in _LOGIC and op not in _MODIFIERS and op not in _EXPRESSIONS:
            return expr
        return expr[end:].lstrip(_BLANKS)

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

        Raises
        ------
        ValueError
            If there is no name.

        """
        node = Node()
        node.node_type = Node.PREDICATE
        pred = predicate
        if pred.startswith('('):
            pred = pred[1:]
        if pred.endswith(')'):
            pred = pred[:-1]
        tokens = _split_blanks(pred)
        if not tokens:
            raise ValueError('Empty predicate')
        node.name = tokens[0]
        for token in tokens[1:]:
            node.parameters.append(Parser.param_from_string(token))  # type: ignore[union-attr]
        node.value = 0.0
        return node

    @staticmethod
    def node_from_string_function(function: str) -> Node:
        """
        Create a Node from a function string.

        Parameters
        ----------
        function : str
            '(f a b)', '(f a b) 1.5' or '(= (f a b) 1.5)'; the value of an object fluent is an
            object: '(= (pos r1) kitchen)'.

        Returns
        -------
        Node
            Node message with FUNCTION type, its parameters and its value or object.

        """
        node = Node()
        node.node_type = Node.FUNCTION
        node.value = 0.0
        # The tokens, and whether each one is inside the parentheses of the function
        tokens = []
        depth = 0
        function_depth = -1
        pos = 0
        while pos < len(function):
            char = function[pos]
            if char == '(':
                depth += 1
                pos += 1
                continue
            if char == ')':
                if depth == function_depth:
                    function_depth = -2  # Closed: what follows is the value
                depth -= 1
                pos += 1
                continue
            if char in _BLANKS:
                pos += 1
                continue
            token = _token_at(function, pos)
            pos += len(token)
            if token == '=' and not tokens:
                continue
            if function_depth == -1:
                function_depth = depth  # The name opens the function
            tokens.append((token, function_depth >= 0))
        if tokens:
            node.name = tokens[0][0]
        for token, inside in tokens[1:]:
            value = Parser.parse_number(token)
            if value is not None:
                node.value = value
                break
            if not inside:
                node.object = token  # After the function: its object
                break
            node.parameters.append(Parser.param_from_string(token))  # type: ignore[union-attr]
        return node

    @staticmethod
    def _node_from_string_quantifier(text: str, node_type: int) -> Node:
        # '- type' applies to the variables before it
        node = Node()
        node.node_type = node_type
        node.value = 0.0
        quantifier, position = _head_token(text)
        open_list = text.find('(', position + len(quantifier))
        close_list = -1 if open_list == -1 else text.find(')', open_list)
        if close_list == -1:
            return node
        tokens = _split_blanks(text[open_list + 1:close_list])
        untyped = 0
        i = 0
        while i < len(tokens):
            if tokens[i] == '-' and i + 1 < len(tokens):
                for k in range(untyped, len(node.parameters)):
                    node.parameters[k].type = tokens[i + 1]
                untyped = len(node.parameters)
                i += 2
            else:
                node.parameters.append(  # type: ignore[union-attr]
                    Parser.param_from_string(tokens[i]))
                i += 1
        return node

    @staticmethod
    def node_from_string_exists(exists: str) -> Node:
        """
        Create a Node from an exists expression.

        Parameters
        ----------
        exists : str
            The exists expression, e.g. '(exists (?a ?b - room) (body))'.

        Returns
        -------
        Node
            Node message with EXISTS type; '- type' applies to the variables before it.

        """
        return Parser._node_from_string_quantifier(exists, Node.EXISTS)

    @staticmethod
    def node_from_string_forall(forall: str) -> Node:
        """
        Create a Node from a forall expression.

        Parameters
        ----------
        forall : str
            The forall expression string, e.g. '(forall (?a ?b - robot) (body))'.

        Returns
        -------
        Node
            Node message with FORALL type; '- type' applies to the variables before it.

        """
        return Parser._node_from_string_quantifier(forall, Node.FORALL)

    @staticmethod
    def metric_from_string(metric: str) -> Metric:
        """
        Create a Metric from a string, as the C++ metricFromString.

        Parameters
        ----------
        metric : str
            'minimize <expr>' or 'maximize <expr>', optionally as '(:metric ...)'. The
            expression is arithmetic over numbers, functions and (total-time).

        Returns
        -------
        Metric
            The metric; its tree is empty if the string is not a valid metric.

        """
        ret = Metric()
        blanks = ' \t\r\n'
        text = metric.strip(blanks)
        # (:metric minimize <expr>) -> minimize <expr>
        if text.startswith('('):
            inner = text[1:].strip(blanks)
            if inner.startswith(':metric') and text.endswith(')'):
                text = inner[len(':metric'):-1].strip(blanks)
        split = min((text.find(c) for c in blanks if c in text), default=-1)
        if split == -1:
            return ret
        optimization = text[:split]
        expr = text[split:].strip(blanks)
        if optimization not in ('minimize', 'maximize'):
            return ret
        tree = Parser.tree_from_string(expr, parent=Node.EXPRESSION)
        if not tree.nodes or not Parser.validate_tree(tree)[0]:
            return ret
        # Only numbers, functions and arithmetic: a comparison or a predicate is not a metric
        arithmetic = (Node.ARITH_ADD, Node.ARITH_SUB, Node.ARITH_MULT, Node.ARITH_DIV)
        for node in tree.nodes:
            if node.node_type == Node.EXPRESSION:
                if node.expression_type not in arithmetic:
                    return ret
            elif node.node_type not in (Node.FUNCTION, Node.NUMBER):
                return ret
        ret.optimization = Metric.MAXIMIZE if optimization == 'maximize' else Metric.MINIMIZE
        ret.tree = tree
        return ret

    @staticmethod
    def metric_to_string(metric: Metric) -> str:
        """
        Convert a Metric to 'minimize <expr>' or 'maximize <expr>'.

        Parameters
        ----------
        metric : Metric
            The metric.

        Returns
        -------
        str
            The metric as a string; empty if there is no metric.

        """
        if not metric.tree.nodes:
            return ''
        optimization = 'maximize' if metric.optimization == Metric.MAXIMIZE else 'minimize'
        return f'{optimization} {Parser.tree_to_string(metric.tree)}'

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
            Tree message; empty if the expression is malformed.

        """
        tree = Tree()
        nesting = Parser.get_max_nesting(expr)
        if nesting < 0 or nesting > MAX_NESTING_DEPTH:
            return tree
        try:
            Parser._from_string_recursive(tree, expr, negate, parent)
        except ValueError:
            tree.nodes = []
        return tree

    @staticmethod
    def _new_node(tree: Tree, node_type: int, negate: bool) -> Node:
        node = Node()
        node.node_type = node_type
        node.node_id = len(tree.nodes)
        node.negate = negate
        tree.nodes.append(node)  # type: ignore[union-attr]
        return node

    @staticmethod
    def _add_children(tree: Tree, node: Node, expr: str, negate: bool) -> None:
        for subexpr in Parser.get_sub_expr(expr):
            child = Parser._from_string_recursive(tree, subexpr, negate, node.node_type)
            if child is not None:
                node.children.append(child.node_id)

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
            The created node, or None for an empty (and) or (not).

        Raises
        ------
        ValueError
            If the expression is malformed.

        """
        wexpr = Parser.get_reduced_string(expr)
        if wexpr in ('(and)', '(and )'):
            return None
        # At the root, as in a goal or a condition, a name is a predicate
        default_node_type = Node.PREDICATE
        if parent in (Node.EXPRESSION, Node.FUNCTION_MODIFIER):
            default_node_type = Node.FUNCTION
        node_type = Parser.get_node_type(wexpr, default_node_type)

        if node_type in (Node.AND, Node.OR, Node.IMPLY):
            node = Parser._new_node(tree, node_type, negate)
            Parser._add_children(tree, node, wexpr, negate)
            return node
        if node_type == Node.WHEN:
            # Never negated: a condition and an effect, each as written
            node = Parser._new_node(tree, node_type, False)
            Parser._add_children(tree, node, wexpr, False)
            return node
        if node_type in (Node.PREFERENCE, Node.CONSTRAINT):
            # Its conditions as written; the times of a constraint are its first children
            op, args = Parser.split_modal(wexpr)  # type: ignore[misc]
            node = Parser._new_node(tree, node_type, negate)
            if node_type == Node.PREFERENCE:
                node.name = args[0] if len(args) == 2 else ''
                args = args[-1:]
            else:
                node.name = op
            for arg in args:
                child = Parser._from_string_recursive(tree, arg, False, node_type)
                if child is not None:
                    node.children.append(child.node_id)
            return node
        if node_type == Node.NOT:
            node = Parser._new_node(tree, node_type, negate)
            subexprs = Parser.get_sub_expr(wexpr)
            child = None
            if subexprs:
                child = Parser._from_string_recursive(tree, subexprs[0], not negate, node_type)
            if child is None:
                # A (not) with nothing to negate is dropped as a whole
                tree.nodes.pop()
                return None
            node.children.append(child.node_id)
            return node
        if node_type == Node.PREDICATE:
            node = Parser.node_from_string_predicate(wexpr)
            node.node_id = len(tree.nodes)
            node.negate = negate
            tree.nodes.append(node)  # type: ignore[union-attr]
            return node
        if node_type == Node.FUNCTION:
            if not wexpr.startswith('('):
                # A bare name is an object, as in (= ?r r2d2): functions go in parentheses
                if not Parser.is_valid_name(wexpr):
                    raise ValueError(f'Wrong name [{wexpr}]')
                node = Parser._new_node(tree, Node.CONSTANT, negate)
                node.name = wexpr
                return node
            node = Parser.node_from_string_function(wexpr)
            node.node_id = len(tree.nodes)
            node.negate = negate
            tree.nodes.append(node)  # type: ignore[union-attr]
            return node
        if node_type == Node.EXPRESSION:
            node = Parser._new_node(tree, node_type, negate)
            node.expression_type = Parser.get_expr(wexpr)[0]
            Parser._add_children(tree, node, wexpr, False)
            return node
        if node_type == Node.FUNCTION_MODIFIER:
            node = Parser._new_node(tree, node_type, negate)
            node.modifier_type = Parser.get_fun_mod(wexpr)[0]
            Parser._add_children(tree, node, wexpr, False)
            return node
        if node_type == Node.NUMBER:
            value = Parser.parse_number(wexpr)
            if value is None:
                raise ValueError(f'Wrong number [{wexpr}]')
            node = Parser._new_node(tree, node_type, negate)
            node.value = value
            return node
        if node_type in (Node.EXISTS, Node.FORALL):
            node = Parser._node_from_string_quantifier(wexpr, node_type)
            node.node_id = len(tree.nodes)
            node.negate = negate
            tree.nodes.append(node)  # type: ignore[union-attr]
            Parser._add_children(tree, node, wexpr, negate)
            return node
        if node_type == Node.PARAMETER:
            # (?x) is ?x
            name = wexpr
            while len(name) > 2 and name.startswith('(') and name.endswith(')'):
                name = Parser.get_reduced_string(name[1:-1])
            node = Parser._new_node(tree, node_type, negate)
            node.name = name
            node.parameters.append(Parser.param_from_string(name))  # type: ignore[union-attr]
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

        Raises
        ------
        ValueError
            If the node is not a predicate or a function.

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
        Convert a Tree to valid PDDL, as the C++ toString.

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
            String representation; empty for an empty or malformed tree.

        """
        if node_id >= len(tree.nodes):
            return ''
        writers = {
            Node.PREDICATE: Parser._to_string_predicate,
            Node.FUNCTION: Parser._to_string_function,
            Node.NUMBER: Parser._to_string_number,
            Node.AND: Parser._to_string_and,
            Node.OR: Parser._to_string_or,
            Node.NOT: Parser._to_string_not,
            Node.EXPRESSION: Parser._to_string_expression,
            Node.FUNCTION_MODIFIER: Parser._to_string_function_modifier,
            Node.CONSTANT: Parser._to_string_constant,
            Node.PARAMETER: Parser._to_string_parameter,
            Node.EXISTS: Parser._to_string_exists,
            Node.FORALL: Parser._to_string_forall,
            Node.IMPLY: Parser._to_string_imply,
            Node.WHEN: Parser._to_string_when,
            Node.PREFERENCE: Parser._to_string_preference,
            Node.CONSTRAINT: Parser._to_string_constraint,
        }
        writer = writers.get(tree.nodes[node_id].node_type)  # type: ignore[index]
        return writer(tree, node_id, negate) if writer else ''

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
            Ignored: a function is never negated.

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
        return result + ')'

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
            Ignored: a number is never negated.

        Returns
        -------
        str
            The number with 6 decimals, or more if they are needed to keep its value.

        """
        if node_id >= len(tree.nodes):
            return ''
        return Parser.number_to_string(tree.nodes[node_id].value)  # type: ignore[index]

    @staticmethod
    def _to_string_junction(tree: Tree, node_id: int, negate: bool, junction: str) -> str:
        if node_id >= len(tree.nodes):
            return ''
        node = tree.nodes[node_id]  # type: ignore[index]
        if not node.children:
            return ''
        if negate:
            junction = 'or' if junction == 'and' else 'and'
        children = ''.join(Parser.tree_to_string(tree, c, negate) for c in node.children)
        return f'({junction} {children})'

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
            Whether to negate: (not (and a b)) is written (or (not a)(not b)).

        Returns
        -------
        str
            String representation; empty without children.

        """
        return Parser._to_string_junction(tree, node_id, negate, 'and')

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
            Whether to negate: (not (or a b)) is written (and (not a)(not b)).

        Returns
        -------
        str
            String representation; empty without children.

        """
        return Parser._to_string_junction(tree, node_id, negate, 'or')

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
            String representation; empty without children.

        """
        if node_id >= len(tree.nodes):
            return ''
        node = tree.nodes[node_id]  # type: ignore[index]
        if not node.children:
            return ''
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
            Whether to negate; only a comparison is negated, never its operands.

        Returns
        -------
        str
            String representation; empty if malformed.

        """
        if node_id >= len(tree.nodes):
            return ''
        node = tree.nodes[node_id]  # type: ignore[index]
        if len(node.children) < 2:
            return ''
        operators = {value: key for key, value in _EXPRESSIONS.items()}
        if node.expression_type not in operators:
            return ''
        negated = negate and node.expression_type in _COMPARISONS
        operands = ''.join(
            ' ' + Parser.tree_to_string(tree, c, False) for c in node.children)
        result = f'({operators[node.expression_type]}{operands})'
        return f'(not {result})' if negated else result

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
            Ignored: an effect is never negated.

        Returns
        -------
        str
            String representation; with only the value, the function is (total-cost).

        """
        if node_id >= len(tree.nodes):
            return ''
        node = tree.nodes[node_id]  # type: ignore[index]
        if not node.children or len(node.children) > 2:
            return ''
        modifiers = {value: key for key, value in _MODIFIERS.items()}
        if node.modifier_type not in modifiers:
            return ''
        result = '(' + modifiers[node.modifier_type]
        if len(node.children) == 1:
            result += ' (total-cost)'
        for child_id in node.children:
            result += ' ' + Parser.tree_to_string(tree, child_id, False)
        return result + ')'

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
            Ignored.

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
            Ignored.

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
    def _to_string_quantifier(tree: Tree, node_id: int, negate: bool, quantifier: str) -> str:
        if node_id >= len(tree.nodes):
            return ''
        node = tree.nodes[node_id]  # type: ignore[index]
        if not node.children:
            return ''
        params = []
        for param in node.parameters:
            # object, the type of untyped variables, is left out: the text stays as written
            if param.type and param.type != 'object':
                params.append(f'{param.name} - {param.type}')
            else:
                params.append(param.name)
        body = ''.join(Parser.tree_to_string(tree, c, False) for c in node.children)
        result = f'({quantifier} ({" ".join(params)}) {body})'
        return f'(not {result})' if negate else result

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
            String representation; untyped variables are of type object.

        """
        return Parser._to_string_quantifier(tree, node_id, negate, 'exists')

    @staticmethod
    def _to_string_forall(tree: Tree, node_id: int, negate: bool) -> str:
        """
        Convert forall node to string.

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
            String representation; untyped variables are of type object.

        """
        return Parser._to_string_quantifier(tree, node_id, negate, 'forall')

    @staticmethod
    def _to_string_imply(tree: Tree, node_id: int, negate: bool) -> str:
        """
        Convert imply node to string.

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
        if len(node.children) != 2:
            return ''
        left = Parser.tree_to_string(tree, node.children[0], False)
        right = Parser.tree_to_string(tree, node.children[1], False)
        result = f'(imply {left} {right})'
        return f'(not {result})' if negate else result

    @staticmethod
    def _to_string_preference(tree: Tree, node_id: int, negate: bool) -> str:
        """
        Convert a preference to string: '(preference name condition)'.

        Parameters
        ----------
        tree : Tree
            The tree structure.
        node_id : int
            Node identifier.
        negate : bool
            Whether to negate it.

        Returns
        -------
        str
            String representation.

        """
        node = tree.nodes[node_id]  # type: ignore[index]
        if len(node.children) != 1:
            return ''
        name = f'{node.name} ' if node.name else ''
        ret = f'(preference {name}{Parser.tree_to_string(tree, node.children[0], False)})'
        return f'(not {ret})' if negate else ret

    @staticmethod
    def _to_string_constraint(tree: Tree, node_id: int, negate: bool) -> str:
        """
        Convert a modal constraint to string: '(within 10 condition)'.

        Parameters
        ----------
        tree : Tree
            The tree structure.
        node_id : int
            Node identifier.
        negate : bool
            Whether to negate it.

        Returns
        -------
        str
            String representation.

        """
        node = tree.nodes[node_id]  # type: ignore[index]
        args = ''.join(' ' + Parser.tree_to_string(tree, child, False) for child in node.children)
        ret = f'({node.name}{args})'
        return f'(not {ret})' if negate else ret

    @staticmethod
    def _to_string_when(tree: Tree, node_id: int, negate: bool) -> str:
        """
        Convert a conditional effect to string: '(when condition effect)'.

        Parameters
        ----------
        tree : Tree
            The tree structure.
        node_id : int
            Node identifier.
        negate : bool
            Not used: a conditional effect is never negated.

        Returns
        -------
        str
            String representation.

        """
        del negate
        if node_id >= len(tree.nodes):
            return ''
        node = tree.nodes[node_id]  # type: ignore[index]
        if len(node.children) != 2:
            return ''
        condition = Parser.tree_to_string(tree, node.children[0], False)
        effect = Parser.tree_to_string(tree, node.children[1], False)
        return f'(when {condition} {effect})'

    @staticmethod
    def compare_str_case_insensitive(first: str, second: str) -> bool:
        """
        Compare two names ignoring the case, as PDDL does.

        Parameters
        ----------
        first : str
            A name.
        second : str
            Another name.

        Returns
        -------
        bool
            Whether they are equal ignoring the case of ASCII letters.

        """
        return first.translate(_ASCII_LOWER) == second.translate(_ASCII_LOWER)

    @staticmethod
    def check_param_equality(first: Param, second: Param,
                             check_var_params: bool = True) -> bool:
        """
        Compare two parameters by name, ignoring the case.

        Parameters
        ----------
        first : Param
            A parameter.
        second : Param
            Another parameter.
        check_var_params : bool, optional
            Whether a variable (?x) must also match; if not, it matches anything.

        Returns
        -------
        bool
            Whether they are equal.

        """
        if not check_var_params and (first.name.startswith('?') or second.name.startswith('?')):
            return True
        return Parser.compare_str_case_insensitive(first.name, second.name)

    @staticmethod
    def check_node_equality(first: Node, second: Node, check_var_params: bool = True) -> bool:
        """
        Compare two nodes, without their children; names are case insensitive.

        Parameters
        ----------
        first : Node
            A node.
        second : Node
            Another node.
        check_var_params : bool, optional
            Whether variables in the parameters must also match.

        Returns
        -------
        bool
            Whether they are equal.

        """
        if first.node_type != second.node_type:
            return False
        if first.node_type in (Node.ACTION, Node.PREDICATE, Node.FUNCTION):
            if not Parser.compare_str_case_insensitive(first.name, second.name):
                return False
        if first.node_type == Node.EXPRESSION and first.expression_type != second.expression_type:
            return False
        if (first.node_type == Node.FUNCTION_MODIFIER and
                first.modifier_type != second.modifier_type):
            return False
        if first.node_type == Node.NUMBER and abs(first.value - second.value) > 1e-9:
            return False
        if len(first.children) != len(second.children):
            return False
        if len(first.parameters) != len(second.parameters):
            return False
        return all(
            Parser.check_param_equality(a, b, check_var_params)
            for a, b in zip(first.parameters, second.parameters))

    @staticmethod
    def check_tree_equality(first: Tree, second: Tree) -> bool:
        """
        Compare two trees node by node.

        Parameters
        ----------
        first : Tree
            A tree.
        second : Tree
            Another tree.

        Returns
        -------
        bool
            Whether they are equal.

        """
        if len(first.nodes) != len(second.nodes):
            return False
        return all(
            Parser.check_node_equality(a, b) for a, b in zip(first.nodes, second.nodes))

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
            11: 'PARAMETER', 12: 'EXISTS', 27: 'FORALL', 28: 'IMPLY',
            29: 'WHEN', 30: 'PREFERENCE', 31: 'CONSTRAINT'
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
