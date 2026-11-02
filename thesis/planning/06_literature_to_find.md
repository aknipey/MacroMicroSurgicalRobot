# 6. Literature to find

`../latex/references.bib` currently holds the 37 entries from your Thesis A review (copied unchanged). The results
now make claims that need literature the Thesis A review did not cover. Below: per claim, what you need to
cite, plus **candidate sources**.

> ⚠ **None of the candidates below has been added to `references.bib`.** They are well-known works as far as I
> know, but **check every one (authors, year, venue, DOI) in Google Scholar / IEEE Xplore / the UNSW library
> before citing.** Add them to the .bib only after you have the real record (Scholar's "Cite → BibTeX" or the
> publisher's export). Don't cite anything you haven't at least skimmed.

---

## Teleoperation latency and transparency (Ch 2.3, Ch 5, Ch 9)

Claim needs: what lag is acceptable in surgical teleop, and how lag degrades performance.
- Sheridan, "Space teleoperation through time delay: review and prognosis", IEEE Trans. Robotics and Automation, 1993.
- Lawrence, "Stability and transparency in bilateral teleoperation", IEEE Trans. Robotics and Automation, 1993.
- Ferrell, "Remote manipulation with transmission delay", IEEE Trans. Human Factors in Electronics, 1965 (the classic move-and-wait result).
- Anvari et al., "The impact of latency on surgical precision and task completion during robotic-assisted remote telepresence surgery", Computer Aided Surgery, 2005.
- Search: "telesurgery latency threshold task performance", "robotic surgery delay 100 ms 200 ms performance".

## Physiological tremor and filtering (Ch 2.3, Ch 5.5, Ch 9.2)

Claim needs: the tremor band is ~8–12 Hz, surgical systems filter it, and the trade-offs between filter types.
- Riviere, Rader, Thakor, "Adaptive canceling of physiological tremor for improved precision in microsurgery", IEEE Trans. Biomedical Engineering, 1998 (WFLC).
- Elble, "Characteristics of physiologic tremor in young and elderly adults", Clinical Neurophysiology, 2003.
- Casiez, Roussel, Vogel, "1€ filter: a simple speed-based low-pass filter for noisy input in interactive systems", CHI 2012.
- Kalman, "A new approach to linear filtering and prediction problems", J. Basic Engineering, 1960.
- Solà, "Quaternion kinematics for the error-state Kalman filter", arXiv:1711.02508, 2017 (the orientation ESKF).
- Your bib already has: sang2016zero (zero-phase), batty2022transparent.
- Search: "da Vinci tremor filtering 6 Hz", "motion scaling tremor filtering surgical robot".

## Servo-level streaming interfaces (Ch 2.3, Ch 4.2.2, Ch 5)

Claim needs: how MoveIt Servo works, and UR External Control's `speedj`/`servoj` semantics.
- Coleman, Şucan, Chitta, Correll, "Reducing the barrier to entry of complex robotic software: a MoveIt! case study", J. Software Engineering for Robotics, 2014.
- Macenski et al., "Robot Operating System 2: Design, architecture, and uses in the wild", Science Robotics, 2022.
- MoveIt Servo documentation and source (cite the version: moveit_servo 2.12.4, `servo.cpp`) as `@software`/`@misc`.
- Universal Robots, *The URScript Programming Language* (e-Series script manual) for `speedj`/`servoj` parameters; Universal Robots ROS 2 driver and `ur_client_library` (`external_control.urscript`, version 2.15.2).
- Search: "real-time joint streaming UR robot servoj latency", "UR RTDE control latency characterization", "MoveIt Servo teleoperation latency". **If someone has published the 60 % displacement issue or speedj chatter, you must cite it and reframe C1/C2 as confirmation.**

## Singularity handling (Ch 2.4, Ch 6)

- Nakamura & Hanafusa, "Inverse kinematic solutions with singularity robustness for robot manipulator control", J. Dynamic Systems, Measurement, and Control, 1986.
- Wampler, "Manipulator inverse kinematic solutions based on vector formulations and damped least-squares methods", IEEE Trans. Systems, Man, and Cybernetics, 1986.
- Yoshikawa, "Manipulability of robotic mechanisms", Int. J. Robotics Research, 1985.
- Search: "singularity avoidance teleoperation manipulability gradient", "singularity robust teleoperation UR wrist singularity", "directional damping singularity".

## RCM (Ch 2.4, Ch 7)

- Taylor et al., "A telerobotic assistant for laparoscopic surgery", IEEE Engineering in Medicine and Biology Magazine, 1995 (mechanical RCM, LARS).
- Aghakhani, Geravand, Shahriari, Vendittelli, Oriolo, "Task control with remote center of motion constraint for minimally invasive robotic surgery", ICRA 2013.
- Your bib: sadeghian2019constrained.
- Search, **important for the novelty claim C6**: "RCM teleoperation motion scaling insertion depth", "fulcrum effect compensation scaling", "port-constrained teleoperation lever ratio", "variable motion scaling laparoscopic depth". If depth-dependent scaling exists, position your law against it (yours keeps tool0 motion constant and stays bounded at the port).

## Haptics and virtual fixtures (Ch 2.4, Ch 6.3)

- Rosenberg, "Virtual fixtures: perceptual tools for telerobotic manipulation", IEEE VRAIS, 1993.
- Abbott, Marayong, Okamura, "Haptic virtual fixtures for robot-assisted manipulation", Robotics Research (ISRR), 2007.
- 3D Systems OpenHaptics Toolkit programmer's guide (device specs: 3.3 N peak, 0.88 N continuous).
- Search: "Geomagic Touch force output characterization", "Phantom Omni force accuracy", "OpenHaptics ROS 2" (check that nobody has reported the lttng-ust/libnuma issue; if not, C8 stands).

## Friction / stick-slip (Ch 5.7)

- Armstrong-Hélouvry, Dupont, Canudas de Wit, "A survey of models, analysis tools and compensation methods for the control of machines with friction", Automatica, 1994.
- Search: "haptic device gimbal friction", "Phantom Omni friction identification".

## Kinematic calibration and tracking (Ch 2.5, Ch 7.6)

- Roth, Mooring, Ravani, "An overview of robot calibration", IEEE J. Robotics and Automation, 1987.
- Tsai & Lenz, "A new technique for fully autonomous and efficient 3D robotics hand/eye calibration", IEEE Trans. Robotics and Automation, 1989.
- NDI Polaris Vega accuracy specification (product documentation).
- Search: "UR5e UR10e kinematic calibration accuracy", "Universal Robots factory calibration error", "cobot absolute accuracy laser tracker" (gives you a literature number for the expected factory-kinematics error, ~mm, to compare your 1–4 mm pose-dependent offsets against).

## Kinematic couplings and tool exchange (Ch 2.6, Ch 8)

- Slocum, "Design of three-groove kinematic couplings", Precision Engineering, 1992.
- Maxwell (1876) / Kelvin coupling history, via Slocum's review "Kinematic couplings: A review of design principles and applications", Int. J. Machine Tools and Manufacture, 2010.
- Your bib: ko2025tool (15 s automated exchange), hong2025backlash, friedman2013instrument.
- ISO 9409-1 (robot mechanical interfaces) as `@standard`/`@misc`.
- Search: "surgical robot instrument quick change repeatability", "3D printed kinematic coupling repeatability".

## Low-cost / open surgical robots (Ch 2.1)

- Your bib: razjigaev2021snakeraven, torielli2023ros, scott2023macro, seetohul2022snake.
- Search: "open-source surgical robot platform ROS", "Raven II", "dVRK da Vinci Research Kit" (the dVRK is the obvious open-platform comparison; cite its main paper).

---

## Existing bib entries: where each one is likely to be used

| Key | Use in |
|---|---|
| scott2023macro | Ch 1.3, 2.2, 9 (baseline) |
| razjigaev2021snakeraven | Ch 2.2, 4.1.2 |
| burgnerkahrs2015continuum, dupont2022continuum, seetohul2022snake | Ch 2.2 |
| sang2016zero, batty2022transparent, bowman2024intent | Ch 2.3 |
| sadeghian2019constrained | Ch 2.4, 7, 9 (RCM threshold) |
| ko2025tool, hong2025backlash, singh2022tension, friedman2013instrument | Ch 2.6, 8 |
| rajih2017error | Ch 2.4 (safety) |
| torielli2023ros | Ch 2.1 |
| eckhoff2023costs, raza2024transforming, dagnino2024trends, bogue2024developments | Ch 1.1, 2.1 |
| iso14971_2019, iec60601_2005a1_2012, iec62304_2006a1_2015 | Ch 2.4 / 9.6 (regulatory context) |
| davinci_intuitive, standardbots_priceguide | Ch 1.1 (cost), but **replace web sources with peer-reviewed ones where possible** (rubric: "inappropriate reliance on sources not peer reviewed") |
