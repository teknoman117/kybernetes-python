#!/usr/bin/env python

import asyncio
import time
import json
import math

from kybernetes import MotionController, GPS, IMU, normalize_heading

class App():
    def __init__(self):
        self.controller = MotionController.Connection()
        self.gps = GPS.Connection()
        self.imu = IMU.Connection()
        self.completed = False

        Ku = 20
        Pu = 4.5
        #self.sKp = 0.6 * Ku
        #self.sKi = 2 * self.sKp / Pu
        #self.sKd = self.sKp * Pu / 8
        self.sKp = Ku
        self.sKi = 0
        self.sKd = 0
        self.active = False

    async def imu_task(self):
        offset = None
        pE = 0
        I = 0
        previous_heading = 0
        response = 0
        while not self.completed:
            orientation = await self.imu.get_orientation()
            heading = orientation.get_heading()

            # store "offset to north"
            if offset is None:
                offset = heading

            error = normalize_heading(heading - offset)

            # don't wind up pid controller if we're not moving
            if not self.active:
                continue

            delta = normalize_heading(heading - previous_heading)
            previous_heading = heading

            if delta > 0:
                response = response + 1
            elif delta < 0:
                response = response - 1
            print(f'[{time.time()}] delta response = {response}')

            # minimize steering error
            #P = self.sKp * error
            #I = I + self.sKi * error
            #D = self.sKd * (error - pE)
            #response = int(P + I + D)
            #pE = error
            #print(f'[{time.time()} error = {error}, response = {response}, P = {P}, I = {I}, D = {D}', flush = True)

            await self.controller.set_steering(response)

    async def gps_task(self):
        while not self.completed:
            position = await self.gps.get_position()
            print(f'gps = {position}', flush = True)

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
            deadzone_left = 1525,
            deadzone_right = 1525,
            Kp = 0.15,
            Ki = 0.25,
            Kd = 0.01
        )

        # issue throttle set while we have work to do
        while not self.completed:
            print('please arm controller to continue', flush = True)
            await self.controller.wait_until_armable()
            await self.controller.arm()

            # status loop
            while True:
                s = await self.controller.get_status()
                if not s.armed():
                    # we were disarmed (remotely, hopefully)
                    self.active = False
                    await self.controller.wait_for_idle()
                    break
                elif self.completed:
                    # we finished our run!
                    break
                else:
                    # otherwise update speed
                    self.active = True
                    await self.controller.set_throttle_pid(0.33)

        await self.controller.set_throttle_pid(0)

        # wait for sensor loops to finish
        await imu_task
        await gps_task

        # tear down connections
        await self.controller.stop()
        await self.gps.stop()
        await self.imu.stop()

# run asynchronous app
if __name__ == "__main__":
    async def main():
        app = App()
        await app.run()

    asyncio.run(main())
