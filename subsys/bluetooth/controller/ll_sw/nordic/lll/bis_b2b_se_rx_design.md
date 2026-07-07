# BIS Subevent Back-to-Back Reception (SE B2B RX) Design

## Overview

This document describes the experimental `radio_switch_complete_and_b2b_se_rx()`
mechanism for BIS (Broadcast Isochronous Stream) subevent reception. The key
difference from existing `radio_switch_complete_and_b2b_rx()` is that the
SW_SWITCH_TIMER is cleared at RADIO READY (not END), enabling pre-scheduled
back-to-back subevent reception where the received ISO PDU length is unknown.

## Problem

In BIS subevent reception, the receiver must listen at precise intervals
(sub_interval) for each subevent. The current implementation uses
`radio_tmr_start_us()` in the ISR to reactively schedule each subsequent
subevent reception. This approach has two issues:
1. The radio start depends on ISR latency
2. Each subevent reception is independently timed rather than chained

The existing `radio_switch_complete_and_b2b_rx()` cannot be used because it
clears the SW_SWITCH_TIMER at EVENTS_END, and the ISO PDU length (and thus the
END timestamp) is unknown before reception.

## Solution: Timer Clear at READY

Clear the SW_SWITCH_TIMER at EVENTS_READY instead of EVENTS_END. Since the
sub_interval between subevents is a fixed, known value, we can calculate the
timer compare offset from the current subevent's READY timestamp.

### Timing Calculation

```
offset = sub_interval - rx_ready_delay - jitter_us

Where:
  sub_interval  = Fixed time between BIS subevents (from BIGInfo)
  rx_ready_delay = Radio RXEN-to-READY ramp-up time (~40us for 1M PHY)
  jitter_us     = Window widening (±2us active clock jitter)
```

The offset is set via `radio_tmr_tifs_set(offset)` before calling
`radio_switch_complete_and_b2b_se_rx()`.

## Timing Diagrams

### Current Implementation (radio_tmr_start_us in ISR)

```
Subevent N                              Subevent N+1
  |                                        |
  v                                        v
  RXEN  READY  ADDRESS ...data... END   RXEN  READY  ADDRESS ...data... END
  |     |      |                  |     |     |
  |     |      |                  |     |     |
  |<--->|      |                  |     |<--->|
  ramp  |      |                  |     ramp  |
        |      |                  |     ^
        |      |                  |     |
        |      |      ISR runs    |     |
        |      |      |---------->|     |
        |      |                  radio_tmr_start_us()
        |      |                  (reactive, ISR latency dependent)
        |      |
        |<-----|------- sub_interval -------->|
```

### New Implementation (radio_switch_complete_and_b2b_se_rx)

```
Subevent N                              Subevent N+1
  |                                        |
  v                                        v
  RXEN  READY  ADDRESS ...data... END   RXEN  READY  ADDRESS ...data... END
  |     |      |                  |     |     |
  |     |      |                  |     |     |
  |<--->|      |                  |     |<--->|
  ramp  |      |                  |     ramp  |
        |      |                  |     ^
        |      |                  |     |
        | SW_SWITCH_TIMER=0       |     |
        | (cleared at READY)      |     |
        |                         |     |
        |------- offset -------->CC compare fires
        |                        RXEN triggered via PPI
        |                         |
        |<---- sub_interval ------+---->|
        |     - rx_ready_delay          |
        |     - jitter_us              |
        |                              |
        offset = sub_interval - rx_ready_delay - jitter_us
```

### Double-Buffer Toggle Mechanism

The sw_switch double-buffer mechanism uses two sets of timer compare channels
(toggle 0 and toggle 1) to avoid race conditions:

```
Toggle 0 setup         Toggle 1 setup         Toggle 0 setup
for SE N+1             for SE N+2             for SE N+3

Subevent N             Subevent N+1           Subevent N+2
  |                      |                      |
  READY                  READY                  READY
  |                      |                      |
  | timer=0              | timer=0              | timer=0
  | PPI group 0 enabled  | PPI group 1 enabled  | PPI group 0 enabled
  |                      |                      |
  |---CC[0] fires------->|                      |
  |   RXEN via PPI[12]   |---CC[1] fires------->|
  |   group 0 disabled   |   RXEN via PPI[13]   |
  |                      |   group 1 disabled   |
  |                      |                      |
  | ISR: reconfigure     | ISR: reconfigure     |
  | radio for SE N+2     | radio for SE N+3     |
  | setup toggle 1       | setup toggle 0       |
  | for SE N+2           | for SE N+3           |
```

### HCTO (Header Complete Timeout) Integration

```
                    EVENT_TIMER (keeps running, not cleared)
                    |
                    v
  READY --------- time ------------------------------------------>
  |                                                    |
  | SW_SWITCH_TIMER=0                                  |
  |                                                    |
  |---- offset ---> CC compare ---> RXEN              |
  |                                  |                 |
  |                                  READY(N+1)       |
  |                                  |                |
  |                                  |--- HCTO ------>X (timeout)
  |                                  |   (absolute,   |
  |                                  |    on EVENT_TIMER)
  |
  | If no packet received (HCTO fires):
  | - Radio disabled by HCTO
  | - But b2b RXEN for next SE still fires independently
  | - ISR handles the timeout and reconfigures as needed
```

### Comparison: b2b_rx vs b2b_se_rx

```
+---------------------------+-----------------------------------+-----------------------------------+
| Aspect                    | b2b_rx (existing)                 | b2b_se_rx (new)                   |
+---------------------------+-----------------------------------+-----------------------------------+
| Timer clear event         | EVENTS_END                        | EVENTS_READY                      |
| Use case                  | tIFS-based (150us),               | BIS subevents (variable           |
|                           | known PDU length (TX)             | sub_interval, unknown RX length)  |
| CC offset meaning         | tIFS - delays                     | sub_interval - rx_ready_delay     |
|                           |                                   | - jitter                          |
| Timing reference          | End of current PDU                | Start (READY) of current PDU      |
| PDU length dependency     | Yes (END depends on length)       | No (READY is length-independent)  |
| RADIO SHORTS              | READY_START + END_DISABLE         | READY_START + END_DISABLE         |
+---------------------------+-----------------------------------+-----------------------------------+
```

## Implementation Notes

### PPI/DPPI Reconfiguration

After calling `sw_switch(RX, RX, ...)` which sets up END-based timer clear,
`radio_switch_complete_and_b2b_se_rx()` patches the PPI/DPPI configuration:

**DPPI (nRF5340/nRF54):**
```
- Clear: nrf_radio_publish_clear(NRF_RADIO, HAL_NRF_RADIO_TIMER_CLEAR_EVENT_END)
- Set:   nrf_radio_publish_set(NRF_RADIO, NRF_RADIO_EVENT_READY, HAL_SW_SWITCH_TIMER_CLEAR_PPI)
```

**PPI (nRF52, separate timer):**
```
- Reconfigure HAL_SW_SWITCH_TIMER_CLEAR_PPI event endpoint from END to READY
- Reconfigure HAL_SW_SWITCH_GROUP_TASK_ENABLE_PPI event endpoint from END to READY
```

### CC Value Restoration

`sw_switch()` subtracts delay from the CC value set by `radio_tmr_tifs_set()`.
For SE RX, the caller already accounts for all delays in the offset value.
After `sw_switch()` returns, the original CC value is restored.

### Kconfig

Gated by `CONFIG_BT_CTLR_SYNC_ISO_SE_B2B_RX` (experimental, depends on
`BT_CTLR_SYNC_ISO` and `!BT_CTLR_TIFS_HW`).
