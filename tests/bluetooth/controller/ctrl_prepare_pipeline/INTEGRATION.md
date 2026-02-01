# Integration Guide for PR #79444 Tests

## Overview

This document explains how to adapt and use these tests when PR #79444 is merged into Zephyr.

## Current State

**Status**: Tests are written but NOT yet integrated with actual implementation.

**Reason**: PR #79444 has not been merged yet. The current Zephyr codebase uses:
- MFIFO-based prepare pipeline
- `uint8_t *idx` iterator parameter
- FIFO ordering (no automatic ordering by `ticks_at_expire`)

**Tests expect** (from PR #79444):
- Ordered linked list prepare pipeline
- `void **idx` iterator parameter
- Automatic ordering by `ticks_at_expire`
- Resume events at tail

## Integration Steps (After PR #79444 Merge)

### Step 1: Update Function Signatures

The current implementation in `subsys/bluetooth/controller/ll_sw/lll.h` will change:

**Before (Current)**:
```c
void *ull_prepare_dequeue_iter(uint8_t *idx);
```

**After (PR #79444)**:
```c
void *ull_prepare_dequeue_iter(void **idx);
```

Verify this change is present before running tests.

### Step 2: Verify Ordered List Implementation

Check that `subsys/bluetooth/controller/ll_sw/ull.c` contains:

1. Ordered linked list structure (not MFIFO_DEFINE)
2. Ordered insertion in `ull_prepare_enqueue()`
3. Resume events placed at tail logic
4. Updated iterator using `void **`

### Step 3: Build System Integration

The tests currently have a minimal CMakeLists.txt. After PR #79444:

1. **Add required sources**: The tests need the actual ULL prepare pipeline implementation.

   Update `CMakeLists.txt` to include:
   ```cmake
   target_sources(testbinary PRIVATE
     # Existing test sources
     src/main.c
     src/test_ull_prepare_basic.c
     src/test_ull_prepare_ordering.c
     src/test_ull_prepare_iterator.c
     src/test_ull_prepare_edge_cases.c
     
     # Add actual implementation or mocks
     ${ZEPHYR_BASE}/subsys/bluetooth/controller/ll_sw/ull.c
     # OR create a minimal mock that provides just the prepare functions
   )
   ```

2. **Add necessary mocks**: The full `ull.c` has many dependencies. You may need to:
   - Mock ticker functions
   - Mock radio functions
   - Mock mayfly scheduler
   - Provide stub implementations for BT controller init

3. **Consider isolated build**: Best approach is to extract just the prepare pipeline code:
   ```cmake
   # Create a test-specific version with minimal dependencies
   target_sources(testbinary PRIVATE
     src/ull_prepare_impl.c  # Extracted prepare pipeline code
   )
   ```

### Step 4: Test Adaptation

#### Iterator Tests

The iterator tests use `void **idx`:

```c
void *idx = NULL;
struct lll_event *current;

current = ull_prepare_dequeue_iter(&idx);
```

**If PR #79444 interface differs**, update all iterator test code to match.

#### Ordering Tests

Ordering tests assume automatic ordering by `ticks_at_expire`:

```c
// Test expects events enqueued as: 5000, 2000, 8000, 3000, 6000
// To be automatically ordered as: 2000, 3000, 5000, 6000, 8000
```

**Verify** that the implementation actually provides this behavior.

#### Resume Event Tests

Tests assume resume events automatically go to tail:

```c
// Normal event at 1000
// Resume event at 500 (earlier, but should be at tail)
// Expected order: normal events first, then resume events
```

**Verify** this is how PR #79444 implements resume event handling.

### Step 5: Pipeline Initialization

The tests currently assume pipeline auto-initialization. After PR #79444:

1. Check if pipeline needs explicit initialization
2. Add setup/teardown in test fixtures if needed:

```c
static void test_ull_prepare_basic_before(void *f)
{
    // Initialize prepare pipeline
    ull_prepare_init();  // If such function exists
}

static void test_ull_prepare_basic_after(void *f)
{
    // Cleanup pipeline
    ull_prepare_cleanup();  // If such function exists
}
```

### Step 6: Memory Pool Configuration

The tests try to enqueue up to `EVENT_PIPELINE_MAX` events. Ensure:

1. Test environment has enough memory pool
2. `EVENT_PIPELINE_MAX` constant is accessible
3. Pool exhaustion is tested correctly

Update `test_pipeline_full_condition` if:
- Pool allocation changed
- Maximum size changed
- Allocation strategy changed

## Testing Strategy

### Phase 1: Smoke Test
Run one basic test to verify build and linking:
```bash
# Build only
west build -p auto -b native_sim tests/bluetooth/controller/ctrl_prepare_pipeline

# If successful, run basic test
west build -t run -- -test=test_enqueue_empty_pipeline
```

### Phase 2: Iterator Tests
Verify iterator interface works:
```bash
west build -t run -- -test=test_ull_prepare_iterator.*
```

### Phase 3: Ordering Tests
Verify automatic ordering:
```bash
west build -t run -- -test=test_ull_prepare_ordering.*
```

### Phase 4: Full Suite
Run all tests:
```bash
west build -t run
```

## Expected Failures (Before PR #79444)

If you try to run these tests on current Zephyr (without PR #79444):

1. **Compilation errors**:
   - `ull_prepare_dequeue_iter` signature mismatch
   - Cannot convert `void **` to `uint8_t *`

2. **Linking errors**:
   - Missing prepare pipeline functions if not properly linked

3. **Runtime failures**:
   - Ordering tests will fail (MFIFO doesn't order by ticks)
   - Resume-at-tail tests will fail (MFIFO doesn't special-case resume)

## Troubleshooting

### Issue: Compilation fails with "incompatible pointer types"

**Cause**: Iterator still uses `uint8_t *idx`

**Solution**: PR #79444 not merged yet, or test code needs update

### Issue: Ordering tests fail

**Cause**: Implementation still uses MFIFO (FIFO ordering)

**Solution**: Verify PR #79444 ordered list code is present

### Issue: Linking fails with undefined references

**Cause**: `ull.c` not compiled/linked properly

**Solution**: Update CMakeLists.txt to include necessary sources

### Issue: Tests crash or hang

**Cause**: Missing mock implementations for dependencies

**Solution**: Add mocks for ticker, radio, mayfly as needed

## Alternative: Mock-Based Testing

Instead of using real implementation, create mocks:

```c
// tests/bluetooth/controller/ctrl_prepare_pipeline/src/ull_prepare_mock.c

static struct lll_event event_pool[EVENT_PIPELINE_MAX];
static struct lll_event *event_list_head = NULL;
static struct lll_event *event_list_tail = NULL;
static int event_count = 0;

struct lll_event *ull_prepare_enqueue(...) {
    // Mock implementation with ordered insertion
    ...
}

void *ull_prepare_dequeue_get(void) {
    return event_list_head;
}

void *ull_prepare_dequeue_iter(void **idx) {
    // Mock iterator implementation
    ...
}
```

This approach:
- ✅ Doesn't depend on actual implementation
- ✅ Can test interface contract
- ✅ Easier to set up
- ❌ Doesn't test real implementation
- ❌ May not catch implementation bugs

## Success Criteria

Tests are properly integrated when:

1. ✅ All tests compile without errors
2. ✅ All tests link successfully
3. ✅ All tests pass (no failures/skips)
4. ✅ Code coverage >90% for prepare pipeline code
5. ✅ Tests run in CI/CD pipeline
6. ✅ No memory leaks reported
7. ✅ All edge cases pass

## Maintenance

After integration:

1. **Monitor PR #79444**: If implementation changes, update tests
2. **Add platform tests**: Extend for Nordic/OpenISA specific code
3. **Performance tests**: Add benchmarks if needed
4. **Regression tests**: Keep tests running in CI
5. **Documentation**: Update this guide based on actual integration experience

## Contact

For questions about test integration:
- Check PR #79444 comments
- Review Zephyr Bluetooth controller documentation
- Consult with PR #79444 author
