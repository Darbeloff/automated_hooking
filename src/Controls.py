#!/usr/bin/env python2

import numpy as np


class PIDController:
    """A generic (serial) PID Controller
    (ask quinn for a mini-lecture on the difference between parallel and serial controllers)
    The derivative controller additionally has a low-pass filter on it - this prevents
    high-frequency noise from taking over the controller
    """
    def __init__(self, Kp, Ki, Kd, tau=0.1, integrator_bounds=(-20, 20), coupled=False):
        """Accept and initialize parameters. Save current time
        Args:
            Kp, Ki, Kd: coefficients for PID control
            alpha: controls the low-pass filter. Increase to allow higher frequencies through. Restrict to (0, 1]
            integrator_bounds: anti-windup limiter on the integrator. Keeps the integrator from blowing up
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.derivative_filter = LPController(tau)
        self.coupled = coupled

        self.e = 0
        self.lag_out_prev = 0
        self.lead = 0
        self.lag_i = 0

        self.integrator_bounds = integrator_bounds

    def do_control(self, state, target, delta_t):
        """Return the controller output, given a reference and output signal
        Args:
            state: where we are
            target: where we want to be

        >--[P]─┬────────[+]---[Lead]--->
               └─[Ki/s ]─┘
        """

        state = np.array(state)
        target = np.array(target)

        if len(target) != len(state):
            raise Exception("Low Pass Controller requires target state length to match state length")

        # First Stage of PID Control: Proportional Gain
        e = target - state
        p_out = e * self.Kp

        # Second Stage: Lag Compensator (integrator)
        self.lag_i += p_out*delta_t*self.Ki
        self.lag_i = np.clip(self.lag_i, self.integrator_bounds[0], self.integrator_bounds[1])
        lag_out = p_out + self.lag_i

        # Third Stage: Lead Compensator (derivative + low-pass filter)
        self.lead_out = lag_out + self.derivative_filter.do_control((lag_out - self.lag_out_prev), delta_t)
        self.lag_out_prev = lag_out
        
        return self.lead_out

class LPController:
    def __init__(self, tau):
        self.tau = tau
        
    def do_control(self, state, target, delta_t):
        state = np.array(state)
        target = np.array(target)

        if len(target) != len(state):
            raise Exception("Controller requires target state length to match state length")

        return state*(1. - delta_t / self.tau) + target*(delta_t / self.tau)


class RateLimiter:
    def __init__(self, max_rate):
        self.max_rate = max_rate
    
    def do_control(self, state, target, delta_t):
        state = np.array(state)
        target = np.array(target)

        if len(target) != len(state):
            raise Exception("Controller requires target state length to match state length")

        diff = target - state

        diff = np.clip( diff, -max_rate, max_rate)

        return state + diff
