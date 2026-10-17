# Vendored Textual

`textual/` is [Textual](https://github.com/Textualize/textual) 6.2.1, unmodified, as published
on PyPI. It is MIT licensed: see `textual/LICENSE`.

The TUI uses the Textual of the system when it is 6.2.1 or newer (`python3-textual`), and this
one otherwise, as on distributions that ship an older one. Textual needs `rich`, `pygments`,
`platformdirs` and `typing_extensions`, declared in `package.xml`; `markdown_it` is only needed
by its Markdown widget, which the TUI does not use.
