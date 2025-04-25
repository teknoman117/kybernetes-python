# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

class PositionController():
    def __init__(self, Kp = 50, Ki = 1, Kd = 100, limits = (-500, 500)):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.limits = limits
        self.previous_error = 0
        self.target = 0
        self.I = 0
        self.moving = True
    
    def set_target(self, target):
        self.target = target
    
    def set_moving(self, moving=True):
        self.moving = moving
    
    def compute(self, input=None, error=None):
        if error is None:
            error = input - self.target
        P = self.Kp * error
        if self.moving:
            self.I = self.I + self.Ki * error
        D = self.Kd * (error - self.previous_error)
        self.previous_error = error            
        response = int(P + self.I + D)
        
        # limit windup by backfeeding I term
        (minimum, maximum) = self.limits
        if response > maximum:
            response = maximum
            self.I = max(maximum - (P + D), 0)
        elif response < minimum:
            response = minimum
            self.I = min(minimum - (P + D), 0)
        return response