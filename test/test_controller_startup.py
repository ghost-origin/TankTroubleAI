# -*- coding: utf-8 -*-
"""Startup controller tests for zero-speed path acquisition."""
from __future__ import annotations

import math
import os
import sys
import unittest

import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
PYTHON_ROOT = os.path.join(ROOT, "python")
if PYTHON_ROOT not in sys.path:
    sys.path.insert(0, PYTHON_ROOT)

from navigation_bot import Bot


def bare_bot(path):
    bot = Bot.__new__(Bot)
    bot.round_ended = False
    bot.path = path
    bot.wp_idx = 1
    bot.raw_wall = np.zeros((570, 570), dtype=bool)
    bot.raw_wall[:2, :] = True
    bot.raw_wall[-2:, :] = True
    bot.raw_wall[:, :2] = True
    bot.raw_wall[:, -2:] = True
    bot.startup_recovery_mode = ''
    bot.startup_recovery_origin = None
    bot.controller_mode = 'STOP'
    return bot


class StartupControllerTests(unittest.TestCase):
    def test_zero_speed_large_error_accelerates_without_steering(self):
        bot = bare_bot([(100.0, 100.0), (100.0, 160.0)])
        me = {'x':100.0, 'y':100.0, 'angle':0.0, 'vx':0.0, 'vy':0.0}

        action = bot.action(me)['keys']

        self.assertEqual(action, {'up':1, 'down':0, 'left':0, 'right':0})

    def test_normal_speed_uses_regular_steering(self):
        bot = bare_bot([(100.0, 100.0), (100.0, 160.0)])
        me = {'x':100.0, 'y':100.0, 'angle':0.0, 'vx':20.0, 'vy':0.0}

        action = bot.action(me)['keys']

        self.assertEqual(action['up'], 1)
        self.assertEqual(action['right'], 1)
        self.assertEqual(action['down'], 0)

    def test_unsafe_forward_probe_uses_validated_reverse_escape(self):
        bot = bare_bot([(100.0, 100.0), (100.0, 160.0)])
        bot.raw_wall[85:115, 118:125] = True
        me = {'x':100.0, 'y':100.0, 'angle':0.0, 'vx':0.0, 'vy':0.0}

        action = bot.action(me)['keys']

        self.assertEqual(action, {'up':0, 'down':1, 'left':0, 'right':0})

    def test_both_directions_blocked_stays_stopped(self):
        bot = bare_bot([(100.0, 100.0), (100.0, 160.0)])
        bot.raw_wall[85:115, 75:82] = True
        bot.raw_wall[85:115, 118:125] = True
        me = {'x':100.0, 'y':100.0, 'angle':0.0, 'vx':0.0, 'vy':0.0}

        action = bot.action(me)['keys']

        self.assertEqual(action, {'up':0, 'down':0, 'left':0, 'right':0})


if __name__ == '__main__':
    unittest.main()
