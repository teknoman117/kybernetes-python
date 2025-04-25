#!/usr/bin/env python
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

import asyncio
import time
import json
import math

from kybernetes import MotionController, GPS, IMU, normalize_heading

class PIDController():
    def __init__(self, Kp = 50, Ki = 1, Kd = 100, limits = (-500, 500)):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.limits = limits
        self.previous_error = 0
        self.target = 0
        self.I = 0

        # log configuration
        msg = dict()
        msg['timestamp'] = time.time()
        msg['type'] = 'configuration'
        msg['Kp'] = self.Kp
        msg['Ki'] = self.Ki
        msg['Kd'] = self.Kd
        print(json.dumps(msg), flush = True)
    
    def set_target(self, target):
        self.target = target
    
    def compute(self, input):
        error = input - self.target
        P = self.Kp * error
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
            
        msg = dict()
        msg['timestamp'] = time.time()
        msg['type'] = 'pid'
        msg['error'] = error
        msg['response'] = response
        msg['P'] = P
        msg['I'] = self.I
        msg['D'] = D
        print(json.dumps(msg), flush = True)
            
        return response

class App():
    def __init__(self):
        self.controller = MotionController.Connection()
        self.gps = GPS.Connection()
        self.imu = IMU.Connection()
        #self.pid = PIDController(Kp=20, Ki=0, Kd=0)
        self.pid = PIDController(Kp=50, Ki=1, Kd=100)
        self.velocity = 0

    async def imu_task(self):
        offset = None
        while True:
            # get heading
            orientation = await self.imu.get_orientation()
            heading = orientation.get_heading()
            if offset is None:
                offset = heading
            heading = normalize_heading(heading - offset)

            # don't wind up pid controller if we're not moving
            if self.velocity != 0:
                await self.controller.set_steering(self.pid.compute(heading))

    async def gps_task(self):
        while True:
            position = await self.gps.get_position()
            msg = dict()
            msg['timestamp'] = time.time()
            msg['type'] = 'gps'
            msg['lat'] = position.get_latitude()
            msg['lon'] = position.get_longitude()
            msg['dec'] = position.get_declination()
            print(json.dumps(msg), flush = True)

    async def run(self):
        # open motion controller and sensors
        await self.controller.start()
        await self.imu.start()
        await self.gps.start()

        # start sensor tasks
        imu_task = asyncio.create_task(self.imu_task())
        gps_task = asyncio.create_task(self.gps_task())

        # send the configuration packet
        await self.controller.set_configuration(\
            deadzone_forward = 1525,
            deadzone_backward = 1475,
            deadzone_left = 1430,
            deadzone_right = 1430,
            Kp = 0.15,
            Ki = 0.25,
            Kd = 0.01
        )

        # issue throttle set while we have work to do
        while True:
            msg = dict()
            msg['timestamp'] = time.time()
            msg['type'] = 'notification'
            msg['text'] = 'please arm controller to continue'
            print(json.dumps(msg), flush = True)
            await self.controller.wait_until_armable()
            await self.controller.arm()

            # status loop
            while True:
                s = await self.controller.get_status()
                self.velocity = s.motion.Input[0]

                if not s.armed():
                    # we were disarmed (remotely, hopefully)
                    await self.controller.wait_for_idle()
                    break
                else:
                    # otherwise update speed
                    await self.controller.set_throttle_pid(1.00)

# run asynchronous app
if __name__ == "__main__":
    async def main():
        app = App()
        await app.run()

    asyncio.run(main())
