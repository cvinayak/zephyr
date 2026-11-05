# PR 102058 bsim CI fix: resume context

Temporary file, drop before upstreaming. Branch `github_bt_ctlr_next` (pushed as
`github_bt_ctlr_next_fix`), based on upstream main 244c4a8284cb. Fixups are NOT squashed.

## Status
- Done: items 1-8, item 9 bis.ticker_expire_info (fixup commits on top of the branch).
- Open: item 9 audio cap_ac on nrf52 (62 FAIL, mostly "ISO Receive Failure"), not investigated.
- Open: item 10 mesh, see below.
- Before upstreaming: 530430dc0 "Limit connection event reservation to interval" is a new
  commit (no Signed-off-by yet) and must move before d1dc6e660; 00388832e carries a
  Co-authored-by trailer to remove; autosquash fixups, per-commit build, check_compliance,
  rerun bsim on nrf52/nrf5340/nrf54l15.

## Item 10 mesh (nrf52_bsim)
Random-seed sweep (seed passed via -rs, hook patch below), failures per seed count:

| script                    | upstream | PR    |
|---------------------------|----------|-------|
| access_transmit_delayable | 2/10     | 2/10  |
| pb_remote_timeout         | 1/10     | 1/10  |
| prov_oob_auth_ib_pk       | 1/10     | 1/10  |
| reprovisioning            | 0/40     | 13/40 |

- access, provision: seed-flaky on upstream too (timing alignment); not PR regressions.
  Restoring `+EVENT_OVERHEAD_START_US` in ll_adv_enable/ull_scan_enable ticks_anchor
  (removed by 8079c8af8) only shifts timing and makes reprovisioning 10/10 fail: do not.
- reprovisioning (persistence, test_persistence.c:762 "Mod app get failed (err -110)"):
  real regression. Debug (seed 22) shows d_01's last request falls in a ~90 ms window where
  d_00's scanner does not run (radio-synced flash ticker id 8 erase in progress; scan ticker
  expires with lazy 3 after it). Flash driver unchanged; PR shifts timing into that window.
- ec01303c9 bisect hit was a false positive (reverting its `>=` collision boundary: still
  13/40). Rate-based bisect is noisy. 40-seed rate profile (seeds 201-240):

| commit (pos/155)  | fails/40 |
|-------------------|----------|
| d1dc6e660 (42)    | 5        |
| 4478d67ab (61)    | 3        |
| ad6e36ecb (73)    | 3        |
| e693e3fff (84)    | 9        |
| 3ea3edf9a (110)   | 9        |
| 04430e782 (135)   | 9        |

  Next: bisect rate step in ad6e36ecb..e693e3fff (ISO sync/broadcast resume, Ticker Oust
  Expire 0fb100a52 and its users) and the 0->~4/40 step in 244c4a8284cb..d1dc6e660.

## Reproduce
Env: venv, ZEPHYR_BASE, BSIM_OUT_PATH, BSIM_COMPONENTS_PATH, ZEPHYR_TOOLCHAIN_VARIANT=host.

    west twister -j2 --build-only -O /tmp/twpb -p nrf52_bsim/native \
        -s bluetooth.mesh.bsim.image.pst
    git apply mesh_rs_hook.patch   # below
    BOARD=nrf52_bsim/native MESH_RS=<seed> \
        tests/bsim/bluetooth/mesh/tests_scripts/persistence/reprovisioning.sh

Seed hook for tests/bsim/bluetooth/mesh/_mesh_test.sh (do not commit):

```diff
diff --git a/tests/bsim/bluetooth/mesh/_mesh_test.sh b/tests/bsim/bluetooth/mesh/_mesh_test.sh
index 83f1c69ca..dd7c5a26d 100755
--- a/tests/bsim/bluetooth/mesh/_mesh_test.sh
+++ b/tests/bsim/bluetooth/mesh/_mesh_test.sh
@@ -67,7 +67,7 @@ function RunTest(){
 
     Execute \
       ${exe_name} \
-      -v=${verbosity_level} -s=$s_id -d=$idx -sync_preboot -RealEncryption=1 \
+      -v=${verbosity_level} -s=$s_id -d=$idx ${MESH_RS:+-rs=$((MESH_RS+idx))} -sync_preboot -RealEncryption=1 \
       -testid=$testid ${testids["${testid}"]} ${test_options}
     let idx=idx+1
   done
@@ -76,7 +76,7 @@ function RunTest(){
 
   echo "Starting phy with $count devices"
 
-  Execute ./bs_2G4_phy_v1 -v=${verbosity_level} -s=$s_id -D=$count -argschannel -at=35
+  Execute ./bs_2G4_phy_v1 -v=${verbosity_level} -s=$s_id -D=$count ${MESH_RS:+-rs=$MESH_RS} -argschannel -at=35
 
   wait_for_background_jobs
 }
```

## Original plan and per-item notes

### PR 102058 bsim-test failure plan (run 41901536121, head 4ecaf2adf7)

Baseline: BabbleSim on other PRs (main) is green -> all 56 failures are PR regressions.

## Phase 1: deterministic fixes (no investigation needed)
1. Build error (split_low_lat x2, mesh image *low_lat x2, 30 mesh SKIPPED as cascade)
   ticker.c:2766 `force` uninitialized when CONFIG_BT_TICKER_LOW_LAT=y.
   Introduced by a43bce851 "Ticker Start unforce feature". Set `force = 1U` in the
   LOW_LAT branch of ticker_job_op_start(). Fold into a43bce851.
2. conn.split_dual_timer / conn_scan_window_split.sh (x3 platforms)
   Script still runs bs_*_prj_split_conf; 03966f797 renamed prj_split.conf ->
   prj_split_dual_timer.conf. Update exe name (and simulation_id) in script. Fold into 03966f797.
3. conn.split_1ms_pllen "Failed to get lock" (5340 cpunet)
   basic_conn_split_1ms_pllen.sh simulation_id lacks ${BOARD_TS}_ prefix -> collides across
   parallel platforms. Fold into 22a84129f.
4. bap_broadcast_sink on nrf5340bsim/cpuapp: HCI Host Buffer Size (0x0c33) -> 0x12, bt_enable -22.
   hci.c rejects Host ACL MTU < LL_LENGTH_OCTETS_RX_MAX: net-core (hci_ipc) data length max now
   exceeds app BT_BUF_ACL_RX_SIZE. Align app ACL RX size or net-core CTLR_DATA_LENGTH_MAX.

## Phase 2: controller runtime regressions (bisect on nrf52_bsim)
5. LL_ASSERT_OVERHEAD at lll_scan.c:513 (nRF54L15 only): host.adv.periodic.coded,
   samples central_hr_peripheral_hr.phy_coded, edtt hci_test_app ll.1/ll.2.llcp.
   Scan prepare preempt overhead check with XTAL_ADVANCED; suspect nRF54L dual-timer / radio
   timer ISR / ordered-prepare-list commits changing EVENT_OVERHEAD or prepare timing.
6. conn.split_1ms (x3) + split_1ms_pllen (nrf52, nrf54l15): notifications arrive in pairs
   every ~2 ms (213 us / 1770 us), so 1 ms delta filter never counts; test runs past 5 s and
   peripheral PPCP auto-update (40/0/42) trips the check. Root cause: every other 1 ms
   connection event skipped. Suspects: ticker reschedule/drift, close-event-early-when-forced,
   oust/unforce, prepare ordered list.
7. ACL supervision timeouts (reason 0x08): throughput_notify_simplex (x3),
   throughput_no_phy_update (x4). Likely same root cause as 6.
8. CIS (all cis.* incl. new acl_group/legacy_adv variants, nrf52 + 5340): central creates
   CIS 0 then never receives adv report when scanning again (acl_first variants pass).
   Scanner starved while CIS active: check scan ticks_slot_window / ISO vs scan overlap
   commits.
9. ISO receive failures: bis.ticker_expire_info (54l15), audio cap_3/5/6/7/8/9/11
   (nrf52 + 54l15; 54l15 cap_* new vs previous run), audio.bsim cap_handover (5340),
   CIS disconnect 0x08. Likely ISO sync resume/jitter/oust commits; recheck after 6-8.
10. Mesh timing: access (time_delta 2487 ms), provision, persistence. Advertiser/scanner
   latency; recheck after 5-8 before investigating.

## Method
- Apply phase 1, rerun to remove cascades.
- Repro locally: `west twister -p nrf52_bsim/native -s bluetooth.controller.conn.split_1ms`
  (needs BSIM_OUT_PATH/BSIM_COMPONENTS_PATH); `git bisect run` over main..PR for 6 and 8.
- Fixes folded into owning commits (fixup + autosquash), no new fix-on-fix commits.

## Progress (local, not pushed)
- 1,2,3 done (fixups). 6,7 done (fixup d1dc6e660 + new 530430dc0, 27fd8a157 for 00388832e).
- 8 done on nrf52 (all 20 cis scenarios pass):
  - ticker.c fixup ec01303c9: window ticker without drift/jitter always yielded -> scan starved.
  - lll_central_iso.c fixup af97a4cb8: div-by-zero (sub_interval 0), interleaved slot index
    mapping, fully elapsed sequential CIS no LLL bookkeeping (payload count ran ahead,
    legacy_adv_sequential), restore latency on all-elapsed early abort.
  - ull_sched.c fixup e1ebafb9e: jitter CIG reserves only burst ticks_slot -> ACL placed inside
    CIG -> CIS offset > ACL interval -> 0x1e (acl_group).
- 4 done: fixup 1feef6540 (source prj.conf ACL RX 255/TX 251; 0x12 was source app ACL MTU 27 < net core 251).
- 5 done: fixup c8e529995 (nRF54L RESUME 62->92; resume path preempt check START-RESUME+82us > START).

- Item 5 EDTT LL/TIM/ADV/BV-03-C (nrf54l15): AUX_ADV_IND 6 us short of T_MAFS because
  ADV_EXT_IND reservation ends START overhead (153 us) before aux; adv_time_get() now pads
  ext adv reservation to MAX(T_MAFS, START+END). Fixup 35d23ffd7 of c8e529995. EDTT 550/550
  on nrf54l15 and nrf52; host adv/advx/central_hr_peripheral_hr pass on both.
