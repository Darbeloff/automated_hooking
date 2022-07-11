#!/usr/bin/env python2

import numpy as np



class PIDController:
    def __init__(self, P, I, D):
        self.P = P
        self.I = I
        self.D = D

    def do_control(self, state, target, delta_t):
        return [0,0]

class LPController:
    def __init__(self, tau):
        self.tau = tau
        
    def do_control(self, state, target, delta_t):
        state = np.array(state)
        target = np.array(target)

        if len(target) != len(state):
            raise Exception("Low Pass Controller requires target state length to match state length")

        return state*(1. - delta_t / self.tau) + target*(delta_t / self.tau)