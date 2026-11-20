# Commit map (history rewrite of 20 November 2026)

On 20 November 2026 the commit messages of this repository were edited to remove tool attribution lines. The
files, authors and dates of every commit are unchanged, but every commit from 8 October 2026 on has a new hash.

Older hashes still appear where they were recorded at the time: the `code_version` of each run in
`data/recordings/**/META.yaml`, and the commit references in `docs/VERSIONS.md`, `docs/THESIS_APPENDIX.md` and the
results files. Look them up here. A `-dirty` suffix in a recording means the run used uncommitted changes on top
of that commit. The git tags (`v1`–`v8`, `thesis-submission`) were moved with the rewrite and point to the same
code as before.

| Old | New | Date | Subject |
|---|---|---|---|
| `b0db4b0` | `0cf60b4` | 2026-10-08 | Touch teleop: direct pen-to-tool mapping with wrist rotation |
| `3fe9421` | `c4fdb23` | 2026-10-08 | almost basic movement |
| `043c661` | `aeab6b9` | 2026-10-12 | V1: Touch teleop baseline with logging (reconstructed) |
| `e329884` | `c113d1f` | 2026-10-12 | V1: TOUCH_ROTATION_GAIN 20 for the 90 degree runs (reconstructed) |
| `3399b82` | `a171177` | 2026-10-12 | V2: Servo JointTrajectory output to scaled JTC (reconstructed) |
| `af2a1e2` | `6f1c8c9` | 2026-10-12 | motion really nice with no filters but with scaling |
| `d11bf25` | `cdf1bed` | 2026-10-12 | some baselines but may need to be re-records |
| `82de072` | `bd45d78` | 2026-10-14 | reset here for V3 |
| `ccb0ee8` | `9f9c88e` | 2026-10-14 | V4: velocity output option, UR emulator, latency study |
| `5a7b1af` | `80e8a25` | 2026-10-15 | V5 ready - smoothing made and lag reduced |
| `21f20d0` | `97c8198` | 2026-10-15 | V5 docs: tag v5, tuned benchmark runs match this code |
| `b82da72` | `e45061e` | 2026-10-15 | singularity improvements |
| `ab2fafb` | `69e340f` | 2026-10-19 | V6 |
| `1958a4a` | `dc1749b` | 2026-10-19 | V6: singularity guard + RCM mode |
| `a43f58e` | `e01be1c` | 2026-10-27 | V7 |
| `eed73bb` | `7358aa8` | 2026-10-30 | V8: macro/micro switch, tool library, smooth speedj, micro module fixes |
| `1b9b502` | `18bc8b6` | 2026-11-02 | After V8: servoj position output, orphan-driver cleanup, rotation scale, NDI optional |
| `9f4f68e` | `6ead1f7` | 2026-11-02 | Rename ls_thesis -> macro_micro_robot and ur5e -> ur16e (names only) |
| `daed0a3` | `c31fad6` | 2026-11-02 | VERSIONS.md: rename row points at 9f4f68e |
| `cf5492f` | `18b463b` | 2026-11-03 | Arm's own kinematic calibration (ur_calibration_extract) + Session A settings |
| `2aa7d0e` | `ef3c0cf` | 2026-11-03 | wrist_jog.py: scripted wrist jog for the wand hand-eye |
| `028bf5f` | `b4c22f3` | 2026-11-03 | Hand-eye halves split by motion; longer suggested robot-tcp recording |
| `27da39c` | `6823c1e` | 2026-11-03 | Tool 3 V2_holder_only: SimpleHolder wand, hand-eye with calibrated kinematics (default) |
| `f9af572` | `5e490a5` | 2026-11-03 | kinematics_check.py: calibration confirmed on the arm (0.000 mm vs 0.85 mm nominal); docs corrected |
| `d1741d6` | `ce3d882` | 2026-11-03 | VERSIONS: Session A replay results (mixed, see logs REPLAYS_2026-11-03.md) |
| `95f9eac` | `47c0190` | 2026-11-03 | stylus_replay: read motion_scale / rcm_active from pre-rename bags too (RCMOnly replayed at 1:1) |
| `0bddba8` | `04ac4a8` | 2026-11-03 | VERSIONS: RCMOnly redone at 1:5 |
| `0156789` | `e4fe17b` | 2026-11-03 | V9: position output for its first real-arm replay set (otherwise Session A settings) |
| `6105dad` | `6f9c3bf` | 2026-11-03 | replay_batch: RCMOnly last (it ends with RCM on) |
| `977f652` | `570cc81` | 2026-11-03 | V9: trajectory output for the TrajectoryMode replay set |
| `a0fe574` | `15b4137` | 2026-11-03 | set_v9_mode.sh; tool-tip clearance notes |
| `d7819e7` | `d22cc2e` | 2026-11-03 | Next-session checklist (V9b); end-of-session notes; failed hand-eye attempts kept |
| `8a7a707` | `8fbe2aa` | 2026-11-03 | V9 mode: final |
| `a821765` | `a3ef940` | 2026-11-06 | V9b runner (RCM runs only), NDI visibility check; report moved to Reports/ |
| `f3ba27a` | `454c18b` | 2026-11-06 | V9 mode: position |
| `3d0d698` | `4534648` | 2026-11-06 | V9 mode: velocity-unfiltered |
| `1e4d6ee` | `eaba878` | 2026-11-06 | V9 mode: velocity-optimised |
| `3934add` | `ca46621` | 2026-11-06 | run_v9b: stop if External Control drops, only keep completed motions |
| `8bc6a3a` | `d651d95` | 2026-11-06 | run_v9b: read program state from the stack log, no subscribers during motion |
| `bc692ba` | `b6e2553` | 2026-11-06 | V9 mode: final |
| `04422ac` | `ba0b314` | 2026-11-06 | V9b: RCM run record; run_v9b no-tool steps 7-11 |
| `8620d8d` | `5b2e184` | 2026-11-06 | V9 mode: trajectory |
| `9d4f2ee` | `73722d2` | 2026-11-06 | V9 mode: position |
| `983ef1a` | `3717532` | 2026-11-06 | V9 mode: velocity-unfiltered |
| `5041eca` | `ae9bae3` | 2026-11-06 | V9 mode: velocity-optimised |
| `de10918` | `a11663a` | 2026-11-06 | V9 mode: final |
| `29da1f6` | `148b0bb` | 2026-11-06 | Singularity guard: cut the twist, not q' (run-off on the mock); E6 scripted test |
| `0c991ff` | `3883b8b` | 2026-11-06 | E6: TOUCH_SINGULARITY_GUARD = False |
| `ccc8f85` | `538f84d` | 2026-11-06 | E6: TOUCH_SINGULARITY_GUARD = True |
| `2cff6f7` | `3ca63c2` | 2026-11-06 | E6 real-arm results; tool speed metric over 50 ms |
| `be121bf` | `b608d9e` | 2026-11-06 | Micro module recalibrated from four dots (tool 2); E0 analysis and plotting; wrist_jog --wrist3-amplitude |
| `2496f3b` | `5996a8f` | 2026-11-06 | Remove unreferenced rviz configs (a .old backup and a ROS 1 .vcg) |
| `dee5cc7` | `9ca55b4` | 2026-11-06 | Remove dead C++: unused functions, unused locals and includes, commented-out code |
| `23bd7db` | `f2eddbf` | 2026-11-06 | Remove unused Python imports and variables |
| `885f4de` | `004c534` | 2026-11-06 | Tidy .gitignore: drop ROS 1 / catkin entries, add Python caches, stop tracking the plot cache |
| `dc42eea` | `144898d` | 2026-11-06 | Gather Claude-generated notes into claude-notes/ |
| `e0ab0b5` | `10b40d2` | 2026-11-06 | Fix E0 write-up image links after the move |
| `515e4cd` | `7f37e4f` | 2026-11-06 | Share controller and joint constants instead of repeating them; name the parameter getter |
| `0b31104` | `1b99d82` | 2026-11-06 | Teleop: split long methods, extract repeated pose and RCM depth code, reuse clamp_norm |
| `d9b0fff` | `2650aeb` | 2026-11-06 | Launch: start ur_emulator through package_node; android publisher: use self |
| `d28c96e` | `b4bd514` | 2026-11-06 | launch_system.sh: extract the repeated driver, bridge and RViz start blocks |
| `6fe0f41` | `0395e1b` | 2026-11-06 | ManipulatorModule: build the constructor from the setters |
| `e538da8` | `ab46f05` | 2026-11-06 | Rename C++ sources to snake_case under include/macro_micro_robot; clearer local names |
| `d5b6738` | `6611aad` | 2026-11-06 | CMake and package.xml: drop boilerplate comments and per-target dependencies no longer used |
| `edd58ee` | `8cbe8d8` | 2026-11-06 | Comments: remove session notes and history, fix a mislabelled constant and a stale haptics docstring |
| `d840348` | `02d567f` | 2026-11-06 | Rewrite the README; move the parts list into docs/ |
| `ddeac1f` | `8e2dd9e` | 2026-11-06 | Format the dense analysis scripts (AST unchanged by the formatter pass) |
| `a2eaee6` | `7d196a8` | 2026-11-06 | Fix flake8 findings in the package and add a .flake8 for this repo's style |
| `8082105` | `0fab2f4` | 2026-11-06 | Remove the android pose nodes and everything that only served them |
| `54e9417` | `a70f75e` | 2026-11-06 | launch_system.sh: continue after a wait timeout, as the warning says |
| `21684f2` | `625e477` | 2026-11-06 | micro_module_control: log the distal length deltas under Distal parameters |
| `cba2d3d` | `b3c338d` | 2026-11-06 | launch_system.sh: make the ur16e and full modes work |
| `8003cb4` | `7a7223d` | 2026-11-06 | Add an MIT licence |
| `0a674e9` | `4a5e8b9` | 2026-11-06 | package.xml: set Aidan Knipe as maintainer |
| `9378945` | `8c15507` | 2026-11-06 | Remove ur16e_control and the dependencies only it used |
| `66f7ddc` | `1e44b64` | 2026-11-19 | V9c: run_v9c.sh phase A (output x filter grid, unattended), grid modes in set_v9_mode.sh, replay_batch --yes |
| `a1075b6` | `6f0a77f` | 2026-11-19 | V9c: default tool 3 |
| `17af1e0` | `4cd0c11` | 2026-11-19 | V9 mode: v5 |
| `cefa995` | `f59e366` | 2026-11-19 | V9 mode: final |
| `e3b7f02` | `707e6d2` | 2026-11-19 | run_v9c: mock stack readiness |
| `5f27779` | `a3fafd7` | 2026-11-19 | V9 mode: v5 |
| `f3a9e3a` | `85be152` | 2026-11-19 | V9 mode: final |
| `807ce3c` | `7a8fda7` | 2026-11-19 | run_v9c: phase B (wand): warm-up, 3 hand-eye fits, NDI floor, RCM per mode, fit/perturbation runs, exchange redo, exchange->RCM |
| `738f350` | `9da1eb4` | 2026-11-19 | V9 mode: p5 |
| `c8754d4` | `77a99a3` | 2026-11-19 | V9 mode: final |
| `dfb37e7` | `764994c` | 2026-11-19 | V9 mode: p5 |
| `064ea03` | `8265965` | 2026-11-19 | V9 mode: final |
| `156210c` | `d862252` | 2026-11-19 | V9 mode: p5 |
| `c3658fc` | `1a210c2` | 2026-11-19 | V9 mode: v5 |
| `c07c1cb` | `5c5962b` | 2026-11-19 | V9 mode: p10 |
| `db23c69` | `9de740e` | 2026-11-19 | V9 mode: v10 |
| `bff3dd7` | `5939c59` | 2026-11-19 | V9 mode: p20 |
| `a45bf8e` | `17afb3b` | 2026-11-19 | V9 mode: v20 |
| `d87802b` | `987f51e` | 2026-11-19 | V9 mode: poff |
| `272f632` | `0eacbcb` | 2026-11-19 | V9 mode: voff |
| `2f86304` | `5565c3d` | 2026-11-19 | V9 mode: p5 |
| `cd95923` | `68e509f` | 2026-11-19 | V9 mode: p20 |
| `8839863` | `3e52482` | 2026-11-19 | V9 mode: final |
| `2277050` | `7b92949` | 2026-11-19 | V9 mode: p5 |
| `0a08703` | `761fa85` | 2026-11-19 | V9 mode: v5 |
| `f6fe1c8` | `750ddb9` | 2026-11-19 | V9 mode: p10 |
| `8d24a67` | `af6f050` | 2026-11-19 | V9 mode: v10 |
| `902b10d` | `da1106a` | 2026-11-19 | V9 mode: p20 |
| `d836710` | `9e461ec` | 2026-11-19 | V9 mode: v20 |
| `cb4932f` | `cd41424` | 2026-11-19 | V9 mode: poff |
| `5c712ea` | `390aa57` | 2026-11-19 | V9 mode: voff |
| `cdedf14` | `ebe22d0` | 2026-11-19 | V9 mode: final |
| `cf15c8e` | `ced8a04` | 2026-11-19 | V9c: rerun_v9c_gaps.sh (top up grid cells whose replays have an arm-path or stylus-stream gap > 50 ms) |
| `55794c2` | `08db80d` | 2026-11-19 | V9 mode: v5 |
| `d9825bd` | `e1e469b` | 2026-11-19 | V9 mode: p20 |
| `b4f7f23` | `905f3e7` | 2026-11-19 | V9 mode: poff |
| `a33431a` | `bd924f6` | 2026-11-19 | V9 mode: voff |
| `f05f0ba` | `94d3635` | 2026-11-19 | V9 mode: final |
| `7b2443e` | `16c1577` | 2026-11-19 | run_v9c phase B: wand-only mount (tool 3 SimpleHolder, no coupling) for steps 20-32; coupling fitted at step 33, own hand-eye fit (tool 9) before the exchange->RCM runs |
| `b2bda8a` | `3d78ea3` | 2026-11-19 | V9 mode: p5 |
| `6bb0c29` | `950b406` | 2026-11-19 | V9 mode: final |
| `8edcd3c` | `e5640cb` | 2026-11-19 | run_v9c: start the NDI tracker before the step 20 warm-up |
| `14f1187` | `f322496` | 2026-11-19 | run_v9c: wait for the tracker with the same node lookup replay_batch.sh uses |
| `8dad667` | `7f15754` | 2026-11-19 | V9 mode: p5 |
| `f778e44` | `21ada7f` | 2026-11-19 | V9c: tool 4 wand_D_fit1 (hand-eye fit) |
| `4ebf892` | `76dc919` | 2026-11-19 | V9c: tool 5 wand_D_fit2 (hand-eye fit) |
| `53fd6f3` | `a14b91e` | 2026-11-19 | V9c: tool 6 wand_D_fit3 (hand-eye fit) |
| `fb12dbd` | `5622f13` | 2026-11-19 | V9c: default tool 4 |
| `d1b14ae` | `dd7d8f3` | 2026-11-19 | V9 mode: final |
| `c3c20ea` | `ba9ecf8` | 2026-11-19 | V9 mode: p5 |
| `0459bd1` | `7d968db` | 2026-11-19 | V9 mode: final |
| `4e50979` | `1b10e8c` | 2026-11-19 | V9 mode: p5 |
| `5180c1a` | `93b403f` | 2026-11-19 | V9 mode: final |
| `66185c8` | `41aa43b` | 2026-11-19 | replay_batch.sh: give each failed replay its own _NO_NDI folder |
| `b95c0f1` | `3465ae7` | 2026-11-19 | V9 mode: p5 |
| `e192315` | `bc9bab0` | 2026-11-19 | V9c: default tool 3 |
| `389fa75` | `fdb9141` | 2026-11-19 | V9 mode: final |
| `6b4b871` | `19988b4` | 2026-11-19 | VERSIONS: step 23 camera move keeps the hand-eye fits |
| `89b089e` | `7bc3aed` | 2026-11-19 | V9c: default tool 4 |
| `ec98c9a` | `4ee055f` | 2026-11-19 | V9 mode: p5 |
| `899bc42` | `9327811` | 2026-11-19 | V9 mode: v5 |
| `8573816` | `c2a69b0` | 2026-11-19 | V9 mode: p20 |
| `91fbd3f` | `f14f5f9` | 2026-11-19 | V9 mode: v20 |
| `ababef5` | `87d73e8` | 2026-11-19 | V9 mode: poff |
| `d7b5fee` | `19f4f6d` | 2026-11-19 | V9 mode: voff |
| `fb77f05` | `b60692e` | 2026-11-19 | V9c: default tool 5 |
| `302c5a4` | `0a47507` | 2026-11-19 | V9 mode: p5 |
| `541a16e` | `5ea0bbe` | 2026-11-19 | V9c: default tool 6 |
| `4f477d8` | `769152b` | 2026-11-19 | V9c: perturbed tools 7 (+1 mm lateral) and 8 (+2 mm axial) from tool 4 |
| `94b8858` | `0ab8b8a` | 2026-11-19 | V9c: default tool 7 |
| `fb1d9df` | `969556d` | 2026-11-19 | V9c: default tool 8 |
| `d18057d` | `2e845e7` | 2026-11-19 | docs: thesis supplementary material moved out of the report appendices |
| `0b44aa3` | `98c94de` | 2026-11-19 | thesis: data analysis folders 01-20, calibration files and planning notes |
| `100b8d5` | `2ba9b8e` | 2026-11-19 | V9 mode: final |
| `45df9fc` | `015616a` | 2026-11-19 | run_v9c.sh: EXCHANGE_STILL_S sets the exchange still periods (default 60 s) |
| `2ef53d9` | `0608905` | 2026-11-19 | Latch-offset test: latch_line_fit.py + run_v9c.sh phase L (steps 40-44) |
| `442ea82` | `9fbb413` | 2026-11-19 | V9 mode: p5 |
| `c892884` | `02d8ee7` | 2026-11-19 | V9c: default tool 3 |
| `7e6618a` | `2f20e50` | 2026-11-19 | V9 mode: final |
| `e3f99ca` | `763f076` | 2026-11-19 | V9 mode: p5 |
| `c9391ba` | `5e70773` | 2026-11-19 | V9c: tool 10 wand_L_fit (hand-eye fit) |
| `46b96e1` | `06b915f` | 2026-11-19 | V9c: default tool 10 |
| `462c1f1` | `10cfacd` | 2026-11-19 | V9c: tool 11 wand_L_corrected (latch-line correction of tool 10) |
| `944b899` | `ceefaf2` | 2026-11-19 | V9c: default tool 11 |
| `468fc94` | `9836ef7` | 2026-11-19 | V9c: default tool 10 |
| `ac67ffd` | `e95bb45` | 2026-11-19 | V9c: default tool 11 |
| `04102e7` | `4bb597f` | 2026-11-19 | V9 mode: final |
| `ee8fc91` | `0242530` | 2026-11-19 | V9c: default tool 2 |
| `288cd34` | `55415ee` | 2026-11-19 | Phase L: step 45 (one extra B FastChecking), skip suffixed runs in latch_line_fit.py |
| `8c88e68` | `513d014` | 2026-11-19 | V9c: default tool 11 |
| `15101ee` | `7b1b9d2` | 2026-11-19 | V9 mode: p5 |
| `a7a3d42` | `6b672ec` | 2026-11-19 | V9 mode: final |
| `986e4df` | `0cde5f6` | 2026-11-19 | V9c: default tool 2 |
| `549a789` | `272213f` | 2026-11-19 | Phase L follow-up: steps 46-48 (tool 12 corrected on all four trajectories) |
| `da9e925` | `cf068af` | 2026-11-19 | data_analysis README: index rows for folders 20 and 21 |
| `7f378fa` | `cbcec14` | 2026-11-19 | V9c: tool 12 wand_L_corrected_all4 (tool 10 corrected on all four trajectories) |
| `2a9fd4f` | `874420f` | 2026-11-19 | V9c: default tool 12 |
| `6dabfe2` | `02adc7f` | 2026-11-19 | V9 mode: p5 |
| `c9cdb78` | `e4c9a67` | 2026-11-19 | V9c: default tool 10 |
| `c35f5bd` | `6545a16` | 2026-11-19 | V9 mode: final |
| `4621ce7` | `dc7a3f1` | 2026-11-19 | run_v9c.sh: step 47 resumes, skipping replays already recorded |
| `69bb08e` | `45e9860` | 2026-11-19 | V9 mode: p5 |
| `1a09c0a` | `8175523` | 2026-11-19 | V9c: default tool 12 |
| `b514a24` | `c9862f0` | 2026-11-19 | V9 mode: final |
| `9fbeb02` | `3e090dd` | 2026-11-19 | V9c: default tool 2 |
| `c6db14c` | `6375db9` | 2026-11-19 | Reorganise: lab and calibration scripts in scripts/ subfolders, all offline analysis under thesis/ |
| `3ca7e89` | `84679e2` | 2026-11-19 | README rewrite, analysis index, fewer comments |
| `f6dd10c` | `296b58f` | 2026-11-19 | Merge tidy-2026-11-19 (repo reorganisation) into results-tidy |
| `ea84812` | `f5583ad` | 2026-11-19 | Results in report order, recordings in data/, test scripts removed |
| `5ee70a4` | `2f24b18` | 2026-11-20 | Final report figures and supplementary appendix for submission |
| `f19c316` | `ad1d993` | 2026-11-20 | THESIS_APPENDIX: match the submitted report's numbering and findings |
| `918be83` | `13cd6ac` | 2026-11-20 | Rename RCM trajectories for the report; add Scott's thesis |
| `84a12ba` | `465650e` | 2026-11-20 | Final figure labels, R05 latch-speed fix and appendix wording for submission |
| `e76a19a` | `842a2aa` | 2026-11-20 | R02: leave excluded runs out of the strip plot (figures as in the submitted report) |
| `65bd1df` | `f8828fa` | 2026-11-20 | Merge repo-cleanup: MIT licence, maintainer, snake_case C++ sources, dead code removed |
| `d7c8236` | `48d4e70` | 2026-11-20 | SIM_latency recordings: record the replayed trajectory by file name, not a temporary path |
| `34322c7` | `81c6d43` | 2026-11-20 | docs/references: README saying what the cited thesis is and where the report uses it |
| `1ffd6c8` | `b68a849` | 2026-11-20 | README: link to the Honours-Thesis overview repository |
| `cddc51c` | `0edd4ba` | 2026-11-20 | README: link the forks actually used and ndi_tracking; drop the removed ur16e_control dependency and the untracked haptics note |
