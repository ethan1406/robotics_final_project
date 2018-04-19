#!/usr/bin/env python
import math


class PController:
    def __init__(self, kp, min_output, max_output):
        self.kp = kp
        self.minOutput = min_output
        self.maxOutput = max_output

    def update(self, value, target_value):
        error = target_value - value
        p = self.kp * error
        output = p
        return max(min(output, self.maxOutput), self.minOutput)
