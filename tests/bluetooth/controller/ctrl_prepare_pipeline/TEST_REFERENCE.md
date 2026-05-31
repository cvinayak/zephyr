# Test Case Reference - Quick Lookup

## Test Suite: test_ull_prepare_basic (7 tests)

| Test Name | Purpose | Key Validation |
|-----------|---------|----------------|
| `test_enqueue_empty_pipeline` | Enqueue to empty pipeline | Event properly stored, fields match |
| `test_enqueue_multiple_events` | Enqueue multiple events | All 5 events stored successfully |
| `test_dequeue_get` | Peek at head without removal | Head accessible, not removed |
| `test_dequeue_empty_pipeline` | Dequeue from empty | Returns NULL |
| `test_resume_event_marking` | Resume event flag | `is_resume == 1` |
| `test_aborted_event_marking` | Aborted event flag | `is_aborted == 1` |
| `test_callback_assignments` | Callback pointers | All 3 callbacks assigned correctly |

## Test Suite: test_ull_prepare_ordering (6 tests)

| Test Name | Purpose | Key Validation |
|-----------|---------|----------------|
| `test_ascending_order_insertion` | Events in ascending time order | Order maintained |
| `test_descending_order_insertion` | Events in descending time order | Reordered to ascending |
| `test_mixed_order_insertion` | Random order insertion | Final order is ascending |
| `test_resume_events_at_tail` | Resume events placement | Resume events after all normal events |
| `test_ordering_with_aborted_events` | Aborted event ordering | Non-aborted events maintain order |
| `test_interleaved_resume_and_normal` | Mixed event types | Normal events first, then resume |

## Test Suite: test_ull_prepare_iterator (9 tests)

| Test Name | Purpose | Key Validation |
|-----------|---------|----------------|
| `test_iterator_init_null` | Iterator starts with NULL | Returns first event |
| `test_complete_iteration` | Iterate all events | Count matches enqueued events |
| `test_iterator_termination` | End of list handling | Returns NULL at end |
| `test_iterator_empty_pipeline` | Iterator on empty | Returns NULL immediately |
| `test_iterator_single_event` | Single event iteration | Returns event then NULL |
| `test_iterator_mixed_event_types` | Iterate mixed types | Counts normal/resume/aborted |
| `test_iterator_break_condition` | Early break safety | Can break and restart |
| `test_iterator_parameter_update` | idx parameter changes | idx updated during iteration |

## Test Suite: test_ull_prepare_edge_cases (11 tests)

| Test Name | Purpose | Key Validation |
|-----------|---------|----------------|
| `test_empty_pipeline_operations` | All ops on empty | All return NULL |
| `test_single_element_pipeline` | Operations with 1 event | Correct head and iteration |
| `test_pipeline_full_condition` | Max capacity handling | Enqueue up to EVENT_PIPELINE_MAX |
| `test_tick_wraparound` | UINT32_MAX wraparound | Events near wraparound handled |
| `test_multiple_dequeue_get_calls` | Non-destructive peek | Multiple calls return same head |
| `test_alternating_enqueue_iterate` | Interleaved operations | Operations don't interfere |
| `test_same_tick_values` | Identical ticks | All events stored, count correct |
| `test_zero_tick_values` | Tick value of 0 | Zero tick handled correctly |
| `test_max_tick_value` | UINT32_MAX tick | Max value handled correctly |

---

**Total: 33 Test Cases**
- 7 Basic operations tests
- 6 Ordering behavior tests  
- 9 Iterator functionality tests
- 11 Edge case tests

**Coverage Areas:**
- ✅ Enqueue operations
- ✅ Dequeue operations
- ✅ Iterator interface (void**)
- ✅ Ordering by ticks_at_expire
- ✅ Resume event handling
- ✅ Aborted event handling
- ✅ Boundary conditions
- ✅ Edge cases
