#!/usr/bin/env python

import asyncio
import time

from kybernetes import MotionController, GPS, IMU, Camera, normalize_heading

class Task():
    pass

class BasicDrive(Task):
    def __new__(cls, *args, **kwargs):
        return super().__new__(cls)

    def __init__(self, distance, heading, velocity):
        pass

class App():
    def __new__(cls, *args, **kwargs):
        return super().__new__(cls)

    def __init__(self):
        self.controller = MotionController.Connection()
        self.gps = GPS.Connection()
        self.imu = IMU.Connection()
        self.camera = Camera.Connection()

        # tasks to perform for this run
        self.completed = False
        self.tasks = []

    async def imu_task(self):
        # get one IMU measurement to know our starting heading
        orientation = await self.imu.get_orientation()
        offset = orientation.get_heading()

        while not self.completed:
            orientation = await self.imu.get_orientation()
            heading = normalize_heading(orientation.get_heading() - offset)
            print(f'[{time.time()}] heading = {heading}', flush=True)

            # TODO: call into tasks
            
    async def gps_task(self):
        while not self.completed:
            position = await self.gps.get_position()
            print(f'[{time.time()}] gps = {position}', flush=True)
            
            # TODO: call into tasks
            
    async def camera_task(self):
        while not self.completed:
            fix = await self.camera.get_fix()
            if fix is None:
                continue
            print(f'[{time.time()}] fix = {fix}', flush=True)
            
            # TODO: call into tasks

    async def run(self):
        # start camera
        await self.camera.start()
        camera_task = asyncio.create_task(self.camera_task())

        # start imu
        await self.imu.start()
        imu_task = asyncio.create_task(self.imu_task())
        
        # start gps
        await self.gps.start()
        gps_task = asyncio.create_task(self.gps_task())

        # start motion controller
        await self.controller.start()

        # send the configuration packet to the motion controller
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
        print(f'Starting Run', flush=True)
        while not self.completed:
            # status loop
            while True:
                status = await self.controller.get_status()
                print(f'[{time.time()}] status = {status}')

        # wait for sensor loops to finish
        print('Run Completed', flush=True)
        await camera_task
        await imu_task
        await gps_task
        
        # tear down connections
        await self.camera.stop()
        await self.gps.stop()
        await self.imu.stop()
        await self.controller.stop()

# run asynchronous app
if __name__ == "__main__":
    async def main():
        app = App()
        await app.run()

    asyncio.run(main())
