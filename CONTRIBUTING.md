# Contributing to 3DS-SDR

Thanks for helping improve 3DS-SDR.

## Development philosophy

This project favors **small, testable, incremental changes**. Please avoid broad rewrites or refactors when a focused patch can solve the problem.

In particular:

- preserve the existing scanner state machine unless a bug requires changing it;
- preserve the current rtl_tcp transport and DSP pipelines unless the issue is directly inside them;
- avoid unrelated cleanup in feature/fix pull requests;
- keep `scanner-v21.cfg` compatibility whenever practical;
- explain DSP changes in terms of sample rate, filter behavior, gain, and expected audible/RF effect;
- test on actual New 3DS hardware when possible.

## Building

A devkitPro 3DS environment with devkitARM and libctru is required.

```sh
make clean
make
```

## Bug reports

Please include the exact version/commit and steps to reproduce. For reception or audio reports, include the mode, frequency/service type, displayed signal level, and—when available—a comparison against another receiver monitoring the same transmission.

## Pull requests

Keep patches focused. Describe:

1. the observed problem;
2. the exact code path changed;
3. why the change is minimal;
4. how it was tested;
5. any compatibility or performance impact.

Do not commit generated build products (`build/`, `.elf`, `.3dsx`, maps, listings, or object files).
