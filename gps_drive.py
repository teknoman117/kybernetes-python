#!/usr/bin/env python

import asyncio
import time
import json
import math

from kybernetes import MotionController, GPS, IMU, normalize_heading

HEADING_Kp = 450.0 / 45.0

class App():
    def __new__(cls, *args, **kwargs):
        return super().__new__(cls)

    def __init__(self):
        self.controller = MotionController.Connection()
        self.gps = GPS.Connection()
        self.imu = IMU.Connection()

        # waypoints around Hellman Hollow in SF's Golden Gate Park
        self.targets = [\
            GPS.Position(latitude=37.76866, longitude=-122.48727),
            GPS.Position(latitude=37.76884, longitude=-122.48782),
            GPS.Position(latitude=37.76861, longitude=-122.48845),
            GPS.Position(latitude=37.76884, longitude=-122.48782),
            GPS.Position(latitude=37.7691168, longitude=-122.4874896),
        ]
        self.position = GPS.Position()
        self.tid = 0
        self.heading_to_target = 0.0
        self.distance_to_target = 0.0
        self.speed_to_target = 0
        self.completed = False

    async def imu_task(self):
        offset = None
        while not self.completed:
            orientation = await self.imu.get_orientation()
            heading = orientation.get_heading()

            # store "offset to north"
            if offset is None:
                offset = heading

            # goal is "north"
            heading = normalize_heading(heading - offset)
            error = -normalize_heading(self.heading_to_target - heading)
            response = int(HEADING_Kp * error)
            print(f'imu = {heading} (target = {self.heading_to_target}, response = {response})')
            await self.controller.set_steering(response)

    async def gps_task(self):
        while len(self.targets) > 0:
            target = self.targets[self.tid]
            self.position = await self.gps.get_position()
            self.heading_to_target = self.position.get_heading_to(target, magnetic=False)
            self.distance_to_target = self.position.get_distance_to(target)
            print(f'gps = {self.position}, distance to target = {self.distance_to_target}, heading to target = {self.heading_to_target}')

            # if we're close to the target, switch to the next one
            if self.distance_to_target > 7:
                self.speed_to_target = 400
            elif self.distance_to_target > 1:
                self.speed_to_target = 100
            else:
                self.tid = self.tid + 1
                if self.tid >= len(self.targets):
                    self.tid = 0

        # we're out of coordinates
        self.completed = True

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
            print('please arm controller to continue')
            await self.controller.wait_until_armable()
            await self.controller.arm()
            
            # status loop
            speed = 0
            while True:
                s = await self.controller.get_status()
                if not s.armed():
                    # we were disarmed (remotely, hopefully)
                    await self.controller.wait_for_idle()
                    break
                elif self.completed:
                    # we finished our run!
                    break
                else:
                    # otherwise update speed
                    if speed != self.speed_to_target:
                        speed = self.speed_to_target
                        await self.controller.set_throttle_pid(speed)

        await self.controller.set_throttle_pid(0)
        print('waypoint run completed')

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
