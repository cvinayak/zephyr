# Prepare Pipeline Unit Tests

## ⚠️ Important Note

**These tests are written for the FUTURE implementation described in PR #79444.**

The current Zephyr codebase uses an MFIFO (Multi-producer FIFO) implementation for the prepare pipeline. PR #79444 will replace this with an ordered linked list implementation and change the iterator interface from `uint8_t*` to `void**`.

**These tests will need to be adapted once PR #79444 is merged**, specifically:

1. The iterator tests assume `void **idx` parameter (PR #79444's new interface)
2. The ordering tests assume ordered insertion based on `ticks_at_expire` (new behavior)
3. The tests assume resume events go to tail automatically (new behavior)

## Overview

This test suite provides comprehensive unit tests for PR #79444, which replaces the Bluetooth controller's prepare pipeline implementation from a FIFO-based approach to an ordered linked list.

## Test Organization

The test suite is organized into four main test modules:

### 1. test_ull_prepare_basic.c - Basic Pipeline Operations
Tests fundamental enqueue/dequeue operations:
- **test_enqueue_empty_pipeline**: Validates enqueuing to empty pipeline
- **test_enqueue_multiple_events**: Tests multiple event enqueuing
- **test_dequeue_get**: Tests peeking at head without removal
- **test_dequeue_empty_pipeline**: Tests dequeue on empty pipeline
- **test_resume_event_marking**: Validates resume event marking
- **test_aborted_event_marking**: Tests aborted event marking
- **test_callback_assignments**: Verifies callback assignments

### 2. test_ull_prepare_ordering.c - Ordering Behavior
Tests the ordered insertion behavior based on `ticks_at_expire`:
- **test_ascending_order_insertion**: Events enqueued in ascending order
- **test_descending_order_insertion**: Events enqueued in descending order (should be reordered)
- **test_mixed_order_insertion**: Random order insertion and verification
- **test_resume_events_at_tail**: Critical test - resume events always at tail regardless of time
- **test_ordering_with_aborted_events**: Tests ordering with aborted events
- **test_interleaved_resume_and_normal**: Complex scenario with mixed event types

### 3. test_ull_prepare_iterator.c - Iterator Functionality
Tests the new `void**` iterator interface (changed from `uint8_t*`):
- **test_iterator_init_null**: Iterator initialization with NULL starts at head
- **test_complete_iteration**: Tests iteration through entire list
- **test_iterator_termination**: Validates iterator returns NULL at end
- **test_iterator_empty_pipeline**: Iterator on empty pipeline
- **test_iterator_single_event**: Edge case with single event
- **test_iterator_mixed_event_types**: Iteration through normal, resume, and aborted events
- **test_iterator_break_condition**: Tests safe break when !idx
- **test_iterator_parameter_update**: Validates idx parameter updates

### 4. test_ull_prepare_edge_cases.c - Edge Cases and Boundary Conditions
Tests edge cases and error handling:
- **test_empty_pipeline_operations**: All operations on empty pipeline
- **test_single_element_pipeline**: Operations with single element
- **test_pipeline_full_condition**: Behavior when pipeline reaches max capacity (EVENT_PIPELINE_MAX)
- **test_tick_wraparound**: Tick counter wraparound scenarios
- **test_multiple_dequeue_get_calls**: Validates dequeue_get is non-destructive
- **test_alternating_enqueue_iterate**: Interleaved operations
- **test_same_tick_values**: Multiple events with identical ticks_at_expire
- **test_zero_tick_values**: Edge case with tick value of 0
- **test_max_tick_value**: Edge case with UINT32_MAX tick value

## Key Changes Tested (PR #79444)

### 1. Data Structure Change
- **From**: MFIFO (Multi-producer FIFO)
- **To**: Ordered linked list
- **Impact**: Events now maintain order based on `ticks_at_expire` instead of simple FIFO

### 2. Iterator Interface Change
- **From**: `void *ull_prepare_dequeue_iter(uint8_t *idx)`
- **To**: `void *ull_prepare_dequeue_iter(void **idx)`
- **Impact**: More flexible iterator that can handle pointer-based traversal

### 3. Ordered Insertion
- Non-resume events are inserted in order based on `ticks_at_expire`
- Resume events always placed at tail
- Aborted events can be anywhere but don't affect ordering logic

### 4. Simplified Logic
- Eliminated O(n) loop for finding short prepare events
- Cleaner preemption handling
- More efficient event management

## Correctness Invariants Verified

### Ordering Invariants
1. Non-resume events maintain ascending `ticks_at_expire` order
2. Resume events always placed at tail of list
3. Aborted events preserved but don't affect ordering

### Iterator Contract
1. Starting with `idx = NULL` begins at head
2. Each call advances to next element
3. `idx = NULL` return signals end of list
4. Breaking when `!idx` is safe and correct

### Memory Management
1. Events properly allocated/deallocated
2. No memory leaks in dequeue operations
3. Graceful handling of pool exhaustion

## Building and Running Tests

### Build
```bash
west build -p auto -b native_sim tests/bluetooth/controller/ctrl_prepare_pipeline
```

### Run
```bash
west build -t run
```

### Run with Ztest Options
```bash
west build -t run -- -v  # Verbose output
```

## Test Coverage Goals

- **Target**: >90% coverage of prepare pipeline code paths
- **Focus Areas**:
  - Core enqueue/dequeue operations
  - Ordering logic
  - Iterator functionality
  - Edge cases and error conditions
  - Platform-specific changes (Nordic and OpenISA)

## Implementation Notes

### Mock Requirements
Tests use simple mock callbacks that return success. Real testing may require:
- Mock ticker implementation
- Mock radio hardware
- Mock mayfly scheduler

### Platform-Specific Tests
While basic pipeline tests are platform-agnostic, full testing should include:
- Nordic LLL tests (`lll_disable`, `lll_prepare_resolve`, `preempt`)
- OpenISA LLL tests (similar functions)

These are not included in the current test suite as they require platform-specific mocking infrastructure.

### Test Limitations

1. **No actual dequeue removal**: Current tests focus on enqueue and iteration. Full dequeue testing with `ull_prepare_dequeue(caller_id)` would require:
   - Mayfly scheduler mocking
   - Ticker system mocking
   - More complex setup

2. **No concurrent access testing**: Tests are single-threaded. Real-world testing should verify:
   - Thread safety
   - Interrupt safety
   - Race conditions

3. **No performance testing**: Tests verify correctness but not performance improvements claimed in PR #79444.

## Future Enhancements

1. Add platform-specific tests for Nordic and OpenISA
2. Add memory leak detection tests
3. Add performance benchmarks comparing MFIFO vs ordered list
4. Add stress tests with rapid enqueue/dequeue cycles
5. Add tests for actual radio event handling
6. Add tests under CONFIG_BT_CTLR_LOW_LAT_ULL_DONE

## References

- **PR #79444**: [Bluetooth: Controller: Replace prepare pipeline with ordered list](https://github.com/zephyrproject-rtos/zephyr/pull/79444)
- **Modified Files**:
  - `subsys/bluetooth/controller/ll_sw/lll.h`
  - `subsys/bluetooth/controller/ll_sw/ull.c`
  - `subsys/bluetooth/controller/ll_sw/nordic/lll/lll.c`
  - `subsys/bluetooth/controller/ll_sw/openisa/lll/lll.c`

## Contributing

When adding new tests:
1. Follow existing test naming conventions
2. Include clear documentation of what is being tested
3. Add test to appropriate test module
4. Update this README with new test descriptions
5. Ensure tests are deterministic and don't depend on timing
