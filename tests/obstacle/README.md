# Obstacle regression tests

Run on the host without the Pico SDK or hardware:

```sh
cmake -S tests/obstacle -B build/obstacle-tests
cmake --build build/obstacle-tests
ctest --test-dir build/obstacle-tests --output-on-failure
```

The GPIO, timer, and interrupt stubs exercise the production module through its
public API and registered callbacks. Cases cover detection transitions, missing
echoes, incomplete/late/unmatched edges, delayed polling, overlapping triggers,
clock wraparound, and an interrupt delivered when the sample handoff restores
interrupts. Physical sensor timing still requires hardware or simulator testing.
