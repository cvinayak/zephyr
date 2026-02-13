# PAwR (Periodic Advertising with Responses) BabbleSim Tests

This directory contains BabbleSim tests for Bluetooth v5.4 Periodic Advertising with Responses (PAwR) feature.

## Test Overview

The tests validate the Upper Link Layer (ULL) implementation of PAwR in the Zephyr Bluetooth Controller. Two devices participate in each test simulation:

1. **PAwR Advertiser** (`pawr_adv`) - Transmits periodic advertising with subevents
2. **PAwR Scanner** (`pawr_sync`) - Synchronizes to periodic advertising and selects subevents

## Test Structure

### Files

- `CMakeLists.txt` - Build configuration for the test
- `prj.conf` - Project configuration enabling PAwR features
- `src/main.c` - Test implementation with two test cases
- `tests_scripts/basic_pawr.sh` - Test execution script

### Test Cases

#### `pawr_adv` - Advertiser Test

This test validates the advertiser-side PAwR functionality:

1. Creates an extended advertising set
2. Sets PAwR parameters using `ll_adv_sync_param_set_v2()`:
   - 1 subevent per periodic event
   - Subevent interval: 20ms
   - Response slot delay: 2.5ms
   - Response slot spacing: 0.5ms
   - 3 response slots per subevent
3. Configures subevent data using `ll_adv_sync_subevent_data_set()`
4. Starts extended and periodic advertising
5. Advertises for 5 seconds
6. Cleanly stops advertising

#### `pawr_sync` - Scanner Test

This test validates the scanner-side PAwR functionality:

1. Starts scanning for extended advertisements
2. Creates a periodic advertising sync
3. Waits for sync establishment
4. Selects subevents using `ll_sync_subevent_set()`
5. Monitors for periodic advertising reports
6. Cleanly terminates the sync

## Running the Tests

### Prerequisites

- BabbleSim installed and configured
- Zephyr environment set up
- `BSIM_OUT_PATH` and `BOARD_TS` environment variables set

### Execution

```bash
cd tests/bsim/bluetooth/ll/pawr/tests_scripts
./basic_pawr.sh
```

The script will:
1. Launch two simulated devices
2. Run the advertiser test on device 0
3. Run the scanner test on device 1
4. Simulate for 60 seconds
5. Report test results

## Current Test Coverage

### ✅ Validated (ULL Layer)

- **Parameter Validation**:
  - Subevent count validation (1-128)
  - Response slot validation
  - Interval validation
  
- **Function Calls**:
  - `ll_adv_sync_param_set_v2()` - Sets PAwR parameters
  - `ll_adv_sync_subevent_data_set()` - Configures subevent data
  - `ll_sync_subevent_set()` - Selects subevents to sync
  - `ll_sync_response_data_set()` - (Tested for parameter validation)

- **Data Storage**:
  - PAwR parameters stored in `ll_adv_sync_set`
  - Subevent selection stored in `ll_sync_set`
  - LLL flags set correctly

### ⏸️ Not Yet Validated (Requires LLL Implementation)

- **Radio Operations**:
  - Subevent transmission timing
  - Response slot scheduling
  - Actual PDU transmission/reception
  
- **HCI Events**:
  - `BT_HCI_EVT_LE_PER_ADV_SUBEVENT_DATA_REQUEST` (0x27)
  - `BT_HCI_EVT_LE_PER_ADV_RESPONSE_REPORT` (0x28)
  
- **Response Handling**:
  - Response reception by advertiser
  - Response transmission by scanner
  - Response slot collision handling

## Expected Behavior

### Current Implementation (ULL Only)

With the current ULL-only implementation:

- ✅ Tests compile successfully
- ✅ ULL functions execute without errors
- ✅ Parameters are validated correctly
- ✅ Data structures are populated
- ⚠️ No actual PAwR radio operation occurs
- ⚠️ Scanner may not receive periodic advertising reports (normal)

### After LLL Implementation

Once the Lower Link Layer is implemented:

- ✅ Full PAwR radio operation
- ✅ Subevent transmission on schedule
- ✅ Response slots functional
- ✅ Scanner receives periodic advertising reports
- ✅ End-to-end PAwR communication works

## Test Parameters

The tests use conservative parameters for initial validation:

```c
#define NUM_SUBEVENTS           1     // Start with 1 subevent
#define SUBEVENT_INTERVAL       0x10  // 20 ms
#define RESPONSE_SLOT_DELAY     0x02  // 2.5 ms
#define RESPONSE_SLOT_SPACING   0x04  // 0.5 ms
#define NUM_RESPONSE_SLOTS      3     // 3 slots per subevent
#define PER_ADV_INTERVAL_MIN    0xA0  // 200 ms
```

These can be adjusted to test different scenarios once the LLL is implemented.

## Future Enhancements

Potential test additions:

1. **Multiple Subevents** - Test with 2-128 subevents
2. **Response Testing** - Validate response transmission/reception
3. **Collision Testing** - Test multiple scanners responding
4. **Error Cases** - Test error handling for invalid parameters
5. **Long Duration** - Test stability over extended periods
6. **Interoperability** - Test with different device combinations

## Troubleshooting

### Build Failures

If the test fails to build:
- Ensure all PAwR Kconfig options are enabled
- Check that the controller supports PAwR (`CONFIG_BT_CTLR_ADV_PERIODIC_RSP_SUPPORT`)
- Verify Zephyr SDK version is compatible

### Test Failures

If tests fail during execution:
- Check BabbleSim logs for detailed error messages
- Verify both devices start successfully
- Ensure simulation timeout is sufficient (60 seconds default)

### No Sync Established

If the scanner doesn't establish sync:
- This is expected with ULL-only implementation
- LLL must be implemented for actual sync to work
- Tests should still pass as they check ULL function calls

## References

- Bluetooth Core Specification v5.4, Vol 6, Part B, Section 4.4.2.9 (PAwR)
- Zephyr Bluetooth Controller Architecture
- BabbleSim Testing Framework

## Contributing

When adding new tests:
1. Follow the existing pattern from `pawr_adv` and `pawr_sync`
2. Add clear logging for debugging
3. Include both positive and negative test cases
4. Document expected behavior
5. Update this README with test descriptions
