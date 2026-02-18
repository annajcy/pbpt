# Spec: Iterative Naive Path Tracer

## Goal
Refactor the existing recursive path tracing logic into an iterative loop. This avoids stack overflow issues for very deep paths and provides a cleaner structure for future optimizations (like wave-front path tracing).

## Scope
- Modify `PathIntegrator` to use a loop-based radiance estimation.
- Introduce a `PathState` structure if necessary to manage per-ray state.
- Support all existing features: multiple bounces, Russian Roulette, light sampling.

## Success Criteria
- Iterative implementation matches the output of the recursive implementation within a small tolerance.
- No performance regressions (ideally a slight improvement).
- Code passes all existing and new tests.
