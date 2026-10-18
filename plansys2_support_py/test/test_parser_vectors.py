# Copyright 2026 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""The Python parser gives the same results as the C++ one, over its test vectors."""

import os

from ament_index_python.packages import get_package_share_directory
from plansys2_msgs.msg import Node
from plansys2_support_py.Parser import Parser
import pytest

TYPES = {
    name: getattr(Node, name) for name in (
        'UNKNOWN', 'AND', 'OR', 'NOT', 'PREDICATE', 'FUNCTION', 'EXPRESSION',
        'FUNCTION_MODIFIER', 'NUMBER', 'CONSTANT', 'PARAMETER', 'EXISTS', 'FORALL', 'IMPLY',
        'WHEN', 'PREFERENCE', 'CONSTRAINT')
}
NAMES = {value: name for name, value in TYPES.items()}


def unescape(text):
    out = []
    i = 0
    while i < len(text):
        if text[i] == '\\' and i + 1 < len(text) and text[i + 1] in 'tnr':
            out.append({'t': '\t', 'n': '\n', 'r': '\r'}[text[i + 1]])
            i += 2
        else:
            out.append(text[i])
            i += 1
    return ''.join(out)


def rows(file, columns):
    path = os.path.join(
        get_package_share_directory('plansys2_pddl_parser'), 'test', 'vectors', file)
    with open(path, encoding='utf-8') as f:
        lines = [line.rstrip('\n') for line in f if not line.startswith('#')]
    out = [[unescape(c) for c in line.split('\t')] for line in lines]
    assert out, file
    for row in out:
        assert len(row) == columns, row
    return [pytest.param(*row, id=repr(row[:2])) for row in out]


@pytest.mark.parametrize('parent,text,written,types', rows('round_trip.tsv', 4))
def test_round_trip(parent, text, written, types):
    tree = Parser.tree_from_string(text, parent=TYPES[parent])
    assert Parser.tree_to_string(tree) == written
    assert ' '.join(NAMES[n.node_type] for n in tree.nodes) == types
    assert Parser.validate_tree(tree)[0]
    for i, node in enumerate(tree.nodes):
        assert node.node_id == i
    # What is written reads back the same
    if written:
        again = Parser.tree_from_string(written, parent=TYPES[parent])
        assert Parser.tree_to_string(again) == written


@pytest.mark.parametrize('default,text,node_type', rows('node_types.tsv', 3))
def test_node_types(default, text, node_type):
    assert NAMES[Parser.get_node_type(text, TYPES[default])] == node_type


@pytest.mark.parametrize('text,name,parameters,value', rows('functions.tsv', 4))
def test_functions(text, name, parameters, value):
    function = Parser.node_from_string_function(text)
    assert function.node_type == Node.FUNCTION
    assert function.name == name
    assert ' '.join(p.name for p in function.parameters) == parameters
    assert Parser.number_to_string(function.value, -1) == value


@pytest.mark.parametrize('text,name,parameters,obj', rows('object_functions.tsv', 4))
def test_object_functions(text, name, parameters, obj):
    function = Parser.node_from_string_function(text)
    assert function.name == name
    assert ' '.join(p.name for p in function.parameters) == parameters
    assert function.object == obj


@pytest.mark.parametrize('text,value', rows('numbers.tsv', 2))
def test_numbers(text, value):
    number = Parser.parse_number(text)
    assert (Parser.number_to_string(number, -1) if number is not None else '-') == value


@pytest.mark.parametrize('first,second,equal', rows('equality.tsv', 3))
def test_equality(first, second, equal):
    a = Parser.tree_from_string(first)
    b = Parser.tree_from_string(second)
    assert Parser.check_tree_equality(a, b) == (equal == 'true')
    assert Parser.check_tree_equality(b, a) == (equal == 'true')


def test_nesting_limit():
    deep = '(not ' * 128 + '(p)' + ')' * 128
    assert Parser.tree_to_string(Parser.tree_from_string(deep)) == '(p)'
    assert Parser.get_max_nesting('(and (p) ; (q\n)') == 2
    assert Parser.get_max_nesting('(and (p)') == -1
    assert Parser.get_max_nesting(')(') == -1
    too_deep = '(not ' * 300 + '(p)' + ')' * 300
    assert len(Parser.tree_from_string(too_deep).nodes) == 0


def test_validate_tree():
    tree = Parser.tree_from_string('(and (p)(not (q)))')
    assert Parser.validate_tree(tree) == (True, '')
    tree.nodes[2].children = []
    assert not Parser.validate_tree(tree)[0]
    tree.nodes[2].children = [0]
    assert 'reached twice' in Parser.validate_tree(tree)[1]
    tree.nodes[2].children = [9]
    assert 'out of range' in Parser.validate_tree(tree)[1]
    tree.nodes[2].children = [3]
    assert Parser.validate_tree(tree)[0]
    tree.nodes[1].node_type = 200
    assert 'unknown type' in Parser.validate_tree(tree)[1]
    assert Parser.validate_tree(Parser.tree_from_string('(> (f) 1 2)'))[0] is False


def test_names():
    assert Parser.is_valid_name('r2d2')
    assert Parser.is_valid_name('wp-1_a')
    assert not Parser.is_valid_name('2wp')
    assert not Parser.is_valid_name('')
    assert not Parser.is_valid_name('?x')
    assert Parser.compare_str_case_insensitive('Robot_At', 'robot_at')
    assert not Parser.compare_str_case_insensitive('robot', 'robots')
    first = Parser.param_from_string('?x')
    second = Parser.param_from_string('r1')
    assert not Parser.check_param_equality(first, second)
    assert Parser.check_param_equality(first, second, check_var_params=False)
    assert not Parser.check_param_equality(Parser.param_from_string(''), second)


def test_number_decimals():
    assert Parser.number_to_string(1.5) == '1.500000'
    assert Parser.number_to_string(1.5, 2) == '1.50'
    assert Parser.number_to_string(1.5, 0) == '1.5'
    assert Parser.number_to_string(1e-7) == '0.0000001'
    assert Parser.number_to_string(0.1, -1) == '0.1'
    assert Parser.number_to_string(-2.0, -1) == '-2'
    assert Parser.number_to_string(float('inf')) == 'inf'
