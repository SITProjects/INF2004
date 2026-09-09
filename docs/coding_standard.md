# Coding standard

Project C code follows [BARR-C:2018](https://barrgroup.com/sites/default/files/barr_c_coding_standard_2018.pdf).
The Pico SDK and other third-party code retain their upstream conventions.

- Use C99, four spaces per indent, LF endings, and at most 80 columns.
- Use lowercase module names with underscores, unique in their first eight
  characters; avoid standard-library header names.
- Pair every source with a same-named header, including `main.c`.
- Guard headers against repeated inclusion and expose only necessary interfaces.
- Include the corresponding header first. Keep internal data and helpers private.
- Follow the section order in the project templates; document modules and future
  functions with Doxygen comments.
- Put braces on separate lines and retain them around all control-flow bodies.
- Review naming, types, initialization, control flow, and other semantic rules
  against the complete standard when adding implementations.

Copy `templates/module.h` and `templates/module.c` for a new module. Rename the
files, guard, include, and file comments together, then register the source in
its CMake target.

`.clang-format` handles supported formatting rules only; it does not certify full
BARR-C conformance or insert missing control-flow braces. The local pre-commit
hook checks C formatting using the container's installed clang-format.
