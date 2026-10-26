"""Haptic cues on the Touch stylus, sent to omni_state's NAME/force_feedback.

The driver applies the last force it was sent (N, in the /phantom/state position axes:
x right, y away from the operator, z up) on every 1 kHz servo cycle until a new one
arrives, and adds a little damping of its own. A node that stops publishing therefore
leaves its last force on: always send zero when done.

The RCM walls and detent are not here: touch_server renders them in the Touch's own
1 kHz loop from a scene the teleop sends (a spring computed over ROS arrives too late
and shakes the pen). What is left changes slowly or not with the pen's position: the
singularity push and the taps (RCM on / off, crossing the RCM point).
"""

import numpy as np


def clamp_norm(vector, limit):
    norm = np.linalg.norm(vector)
    return vector * (limit / norm) if norm > limit else vector


def singularity_force(relieve, limiting, velocity, peak, damping):
    """Force (N, 3) pushing the stylus along `relieve` (the stylus direction that moves
    the arm out of the singularity): `peak` x limiting (0 at the guard's slow
    threshold, 1 at stop), plus `damping` x limiting on the part of the stylus
    velocity heading the other way (into the singularity)."""
    norm = np.linalg.norm(relieve)
    if limiting <= 0.0 or not norm > 1e-12:
        return np.zeros(3)
    out = relieve / norm
    into = max(0.0, -float(velocity @ out))
    return limiting * (peak + damping * into) * out


class PulsePlayer:
    """Short tap patterns. A pattern is a list of (start, duration) pulses (s), each a
    Hann-window bump along a direction, so the force starts and ends at 0 (a jump in
    force can trip the Touch's max-force-impulse check). With `frequency` > 0 each pulse
    is a buzz at that frequency instead of a single push. Taps are played open loop (they
    do not depend on where the pen is), so unlike a spring they cannot set the pen
    oscillating."""

    PATTERNS = {
        "rcm_on": [(0.0, 0.06)],                                     # one tap
        "rcm_off": [(0.0, 0.06)],                                    # one tap
        "refused": [(0.0, 0.06), (0.15, 0.06)],                      # two taps
        "crossing": [(0.0, 0.04)],                                   # one short click
        "limit": [(0.0, 0.04)],                                      # one short click
    }

    def __init__(self, amplitude, frequency):
        self.amplitude, self.frequency = amplitude, frequency
        self.bursts, self.start = None, None
        self.vector = np.array([0.0, 0.0, 1.0])

    def play(self, name, now, direction=(0.0, 0.0, 1.0), amplitude=None):
        """Start pattern `name` at time `now` (s) along unit `direction` (default up), at
        `amplitude` N (default the player's), cutting off any pattern still playing."""
        amplitude = self.amplitude if amplitude is None else amplitude
        if amplitude > 0.0:
            self.bursts, self.start = self.PATTERNS[name], now
            self.vector = amplitude * np.asarray(direction, dtype=float)

    def __call__(self, now):
        """Force (N, 3-vector) at time `now` (s)."""
        if self.bursts is None:
            return np.zeros(3)
        t = now - self.start
        if t > max(start + duration for start, duration in self.bursts):
            self.bursts = None
            return np.zeros(3)
        for start, duration in self.bursts:
            local = t - start
            if 0.0 <= local <= duration:
                window = np.sin(np.pi * local / duration) ** 2
                if self.frequency > 0.0:
                    window *= np.sin(2.0 * np.pi * self.frequency * local)
                return window * self.vector
        return np.zeros(3)


def limit_force(force, previous, max_force, max_step):
    """`force` capped at max_force (N) and moved at most max_step (N) from `previous`."""
    force = clamp_norm(np.asarray(force, dtype=float), max_force)
    return previous + clamp_norm(force - previous, max_step)
