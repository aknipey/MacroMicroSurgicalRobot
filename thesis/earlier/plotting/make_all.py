#!/usr/bin/env python3
"""Rebuild every figure in thesis/earlier/plotting/out/ from logs/ (read-only).

    cd ~/ros2_ws/src/MacroMicroSurgicalRobot && python3 thesis/earlier/plotting/make_all.py

The first tracking run computes plot_paths metrics for ~50 replays (~15 min); later runs use the cache.
"""
import rcm_plots
import tracking_plots

if __name__ == '__main__':
    rcm_plots.main([])
    tracking_plots.main()
