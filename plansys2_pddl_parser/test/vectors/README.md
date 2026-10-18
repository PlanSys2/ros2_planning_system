# Parser test vectors

Cases read by the tests of the C++ parser (`vectors_test.cpp`) and of its Python port
(`plansys2_support_py`), so that both read and write expressions the same way. Columns are
separated by tabs; `\t`, `\n` and `\r` in a column stand for those characters, and lines
starting with `#` are comments.

- `round_trip.tsv`: parent type, text, text written back from the tree (empty: no tree), and
  the types of its nodes in order.
- `node_types.tsv`: default type, text, node type.
- `functions.tsv`: text, name, parameters (separated by spaces), value.
- `object_functions.tsv`: text, name, parameters, the object of an object fluent (empty: none).
- `numbers.tsv`: text, value (`-` if it is not a number).
- `equality.tsv`: first text, second text, whether their trees are equal.
