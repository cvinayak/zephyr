# Overhead Time Analysis for LLL Resume

## Question
What is the shortest overhead time from radio end to radio start in this pull request when roles use lll resume?

## Answer
**315 microseconds** (for Nordic platforms with minimal configuration)

## Detailed Explanation

The overhead time from radio end to radio start when using lll_resume is the sum of:

1. **EVENT_OVERHEAD_END_US**: Time after a radio event ends (40 microseconds)
2. **EVENT_OVERHEAD_START_US**: Time before the next radio event starts (varies by configuration)

### Platform-Specific Values

#### Nordic Platforms
The `EVENT_OVERHEAD_START_US` varies based on configuration:
- **275 us**: Basic configuration (without extended advertising features or with basic advertising only)
- **428 us**: Extended advertising with scanning on 1M PHY
- **641 us**: Extended advertising with scanning on 1M and Coded PHY

**Minimum overhead for Nordic**: 40 us + 275 us = **315 us**

#### OpenISA Platforms
- **EVENT_OVERHEAD_START_US**: 300 us (fixed)
- **Total overhead**: 40 us + 300 us = **340 us**

### Overall Minimum
The **shortest overhead time across all platforms is 315 microseconds**, occurring on Nordic platforms with the minimal configuration (no extended advertising features or with only basic advertising).

## Code Reference

The new macro `EVENT_OVERHEAD_RESUME_MIN_US` has been added to both vendor header files:
- `/subsys/bluetooth/controller/ll_sw/nordic/lll/lll_vendor.h`
- `/subsys/bluetooth/controller/ll_sw/openisa/lll/lll_vendor.h`

This macro can be used throughout the codebase when calculating or validating timing constraints for lll_resume operations.

## Related Constants

- `EVENT_OVERHEAD_END_US = 40 us` - Worst-case time margin after event end
- `EVENT_OVERHEAD_START_US = 275-641 us` (Nordic) or 300 us (OpenISA) - Time for event setup
- `EVENT_OVERHEAD_START_MIN_US = 275 us` (Nordic) - Minimum start overhead constant
- `EVENT_OVERHEAD_RESUME_MIN_US = 315 us` (Nordic) or 340 us (OpenISA) - Total minimum overhead
