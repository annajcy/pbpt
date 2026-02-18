# Plan: Iterative Naive Path Tracer

## Phase 1: Infrastructure & Testing Setup
- [ ] Task: Define the iterative loop structure and state management.
    - [ ] Research current recursive implementation in `src/pbpt/integrator/plugin/integrator/path_integrator.hpp`.
    - [ ] Create a unit test in `test/integrator/test_iterative_path_tracer.cpp` that compares recursive vs iterative results for 1-bounce.
- [ ] Task: Conductor - User Manual Verification 'Phase 1: Infrastructure & Testing Setup' (Protocol in workflow.md)

## Phase 2: Iterative Core Implementation
- [ ] Task: Implement the main iterative radiance loop.
    - [ ] Port the direct illumination and BSDF sampling logic into the loop.
    - [ ] Implement Russian Roulette termination within the iterative structure.
- [ ] Task: Support for complex paths (Multiple Bounces).
    - [ ] Ensure throughput and radiance accumulation are correctly handled across multiple bounces.
    - [ ] Verify correctness with simple Cornell Box scenes.
- [ ] Task: Conductor - User Manual Verification 'Phase 2: Iterative Core Implementation' (Protocol in workflow.md)

## Phase 3: Verification & Polish
- [ ] Task: Performance and Accuracy Validation.
    - [ ] Run full-scene integration tests (e.g., Cornell Box 16spp).
    - [ ] Compare performance metrics between old and new implementations.
- [ ] Task: Clean up and final documentation.
    - [ ] Remove legacy recursive code paths once the iterative implementation is proven.
    - [ ] Update documentation and internal comments to reflect the new structure.
- [ ] Task: Conductor - User Manual Verification 'Phase 3: Verification & Polish' (Protocol in workflow.md)
