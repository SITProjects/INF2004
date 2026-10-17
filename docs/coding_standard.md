# Coding standard

Project C code in `app_program/` and `templates/` follows
[BARR-C:2018](https://barrgroup.com/sites/default/files/barr_c_coding_standard_2018.pdf).
The micro T-Kernel sources (`kernel/`, `include/`, `lib/`, `device/`,
`config/`, `etc/`, `build_make/`), the Pico SDK and other third-party code
retain their upstream conventions; edit them only when the team agrees, and
keep their style.

- Use C11 (the build uses `-std=gnu11`), four spaces per indent, LF endings,
  and at most 80 columns.
- Use lowercase module names with underscores, unique in their first eight
  characters where practical; avoid standard-library header names. Prefix
  public symbols with the module name.
- Pair every source with a same-named header. Two template-defined
  exceptions: `app_main.c` uses the team header `app.h`, and each
  `<name>_test.c` declares nothing of its own (its `<name>_test()` is in
  `app.h`).
- Guard headers against repeated inclusion and expose only necessary
  interfaces. Include the corresponding header first. Keep internal data and
  helpers `static`.
- Follow the section order in the project templates; document modules and
  public functions with Doxygen comments, including units, return values,
  initialization requirements, ownership, and which task or core may call
  them.
- Put braces on separate lines and retain them around all control-flow
  bodies.
- Use fixed-width types (`stdint.h`) in module interfaces. Use the T-Kernel
  types (`ER`, `ID`, `INT`, `UW`, ...) where a value goes to or comes from a
  kernel call.
- Do not call Pico SDK runtime functions (`pico/stdlib.h`, `hardware/*.c`,
  `printf`): the kernel build does not link them. Use the kernel API, the
  board helpers in `bsp/libbsp.h`, the kernel device drivers
  (`tk_opn_dev()`), or register access with `in_w()`/`out_w()`. Print with
  `tm_printf()`, which has no `%f`.
- Never call `cyw43_` or lwIP functions from application tasks; see
  `TEAM.md` section 3.
- Review naming, types, initialization, control flow, and other semantic
  rules against the complete standard when adding implementations.

## Starting a new file

- A new subsystem: create `app_program/<name>/` and copy
  `templates/subsystem.h`, `templates/subsystem.c` and
  `templates/subsystem_test.c` into it as `<name>.h`, `<name>.c` and
  `<name>_test.c`. Rename the files, guard, includes, functions and file
  comments together; declare `<name>_init()`, `<name>_start()` and
  `<name>_test()` in `app_program/app.h` and call them from `usermain()` in
  `app_program/app_main.c` (team agreement needed). Add `testing/<name>/`
  and a `TEAM.md` row.
- A driver or feature module inside a subsystem: copy `templates/module.h`
  and `templates/module.c` as `drv_<name>.c/.h` (drivers) or
  `<subsystem>_<what>.c/.h`. Rename the files, guard, include, and file
  comments together.

There is nothing to register: the makefile compiles every `.c` file in
`app_program/` and its first level of subfolders.

## Checks

`.clang-format` handles supported formatting rules only; it does not certify
full BARR-C conformance or insert missing control-flow braces. The pre-commit
hook checks the formatting of `app_program/` and `templates/` with the
container's clang-format. To check by hand:

```sh
find app_program templates -name '*.[ch]' -print0 |
    xargs -0 clang-format --dry-run --Werror
```
