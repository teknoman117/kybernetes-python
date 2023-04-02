#!/usr/bin/env python

import asyncio
import time
import json

from kybernetes import MotionController, GPS, IMU, normalize_heading

HEADING_Kp = 450.0 / 45.0

class App():
    def __new__(cls, *args, **kwargs):
        return super().__new__(cls)

    def __init__(self):
        self.controller = MotionController.Connection()
        self.gps = GPS.Connection()
        self.imu = IMU.Connection()

        # center of a intersection near SF's Golden Gate Park
        self.target = GPS.Position(latitude=37.765456, longitude=-122.477337)
        self.position = GPS.Position()
        self.heading_to_target = 0.0
        self.distance_to_target = 0.0

    async def imu_task(self):
        while True:
            orientation = await self.imu.get_orientation()
            heading = orientation.get_heading()

            # goal is "north"
            error = -normalize_heading(self.heading_to_target - heading)
            response = int(HEADING_Kp * error)
            print(f'imu = {heading} (target = {self.heading_to_target}, response = {response})')
            await self.controller.set_steering(response)

    async def gps_task(self):
        while True:
            self.position = await self.gps.get_position()
            self.heading_to_target = self.position.get_heading_to(self.target, magnetic=True)
            self.distance_to_target = self.position.get_distance_to(self.target)
            print(f'gps = {self.position}, distance to target = {self.distance_to_target}, heading to target = {self.heading_to_target}')

    async def run(self):
        # open motion controller and sensors
        await self.controller.start()
        await self.imu.start()
        await self.gps.start()

        # start sensor tasks
        asyncio.create_task(self.imu_task())
        asyncio.create_task(self.gps_task())

        # send the configuration packet
        await self.controller.set_configuration(\
            deadzone_forward = 1525,
            deadzone_backward = 1475,
            deadzone_left = 1580,
            deadzone_right = 1580,
            Kp = 0.15,
            Ki = 0.25,
            Kd = 0.01
        )
        
        # spin forever
        while True:
            print('arm controller to follow heading')
            await self.controller.wait_until_armable()
            await self.controller.arm()
            print('following heading')
            await self.controller.set_throttle_pid(100)
            await self.controller.wait_for_disarm()
            print('disarming')
            await self.controller.wait_for_idle()

# run asynchronous app
if __name__ == "__main__":
    asyncio.run(App().run())
