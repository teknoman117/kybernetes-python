#!/usr/bin/env python

import asyncio
import time
import math

from kybernetes import MotionController, GPS, IMU, Camera, normalize_heading

def clamp(value, minimum, maximum):
    return max(min(value, maximum), minimum)

class Task():
    pass

DRIVE_STEERING_KP = 50
DRIVE_THROTTLE_KP = 0.75
ALIGN_STEERING_KP = 100
ALIGN_THROTTLE_KP = 1.0 / 10.0

MINIMUM_VELOCITY = 0.3

class Drive(Task):
    def __new__(cls, *args, **kwargs):
        return super().__new__(cls)

    def __init__(self, app, distance, heading, velocity):
        self.app = app
        self.distance = distance
        self.heading = heading
        self.velocity = velocity

        # internal state
        self.stopping = False

    async def on_enter(self):
        print(f'[{time.time()}] Drive.on_enter()', flush=True)
        await self.app.controller.reset_odometer()

    async def on_exit(self):
        print(f'[{time.time()}] Drive.on_exit()', flush=True)

    async def on_imu(self, orientation, heading):
        error = -normalize_heading(self.heading - heading)
        response = int(DRIVE_STEERING_KP * error)
        if self.velocity < 0:
            response = -response
        print(f'[{time.time()}] Drive.on_imu(): heading error = {error}', flush=True)
        await self.app.controller.set_steering(response)

    async def on_gps(self, position):
        pass

    async def on_camera(self, fix):
        pass

    async def on_status(self, status):
        # extract the PID input and add it to the odometer
        meters = status.odometer / MotionController.TICKS_PER_METER
        error = self.distance - meters
        print(f'[{time.time()}] Drive.on_status(): distance error = {error}', flush=True)

        # wait until our motion ceases before calling task_done()
        if self.stopping:
            if status.stopped():
                await self.app.task_done()
            return

        if self.velocity < 0:
            response = clamp(DRIVE_THROTTLE_KP * error, self.velocity, -MINIMUM_VELOCITY)
            if error > -0.05:
                await self.stop()
                return
        else:
            response = clamp(DRIVE_THROTTLE_KP * error, MINIMUM_VELOCITY, self.velocity)
            if error < 0.05:
                await self.stop()
                return
        await self.app.controller.set_throttle_pid(response)

    async def stop(self):
        print(f'[{time.time()}] Drive.stop()', flush=True)
        await self.app.controller.set_throttle_pwm(0)
        self.stopping = True

class Align(Task):
    def __new__(cls, *args, **kwargs):
        return super().__new__(cls)

    def __init__(self, app, heading, velocity, should_stop=True):
        self.app = app
        self.heading = heading
        self.velocity = velocity
        self.should_stop = should_stop

        # internal state
        self.stopping = False

    async def on_enter(self):
        print(f'[{time.time()}] Align.on_enter()', flush=True)

    async def on_exit(self):
        print(f'[{time.time()}] Align.on_exit()', flush=True)

    async def on_imu(self, orientation, heading):
        error = -normalize_heading(self.heading - heading)
        print(f'[{time.time()}] Align.on_imu(): heading error = {error}', flush=True)

        if self.stopping:
            return

        steering_response = int(ALIGN_STEERING_KP * error)
        if self.should_stop:
            throttle_response = clamp(ALIGN_THROTTLE_KP * error, MINIMUM_VELOCITY, abs(self.velocity))
        else:
            # if we don't need to stop, just run at target velocity
            throttle_response = abs(self.velocity)

        if abs(error) < 2.5:
            await self.stop()
            return

        if self.velocity < 0:
            steering_response = -steering_response
            throttle_response = -throttle_response

        await self.app.controller.set_steering(steering_response)
        await self.app.controller.set_throttle_pid(throttle_response)

    async def on_gps(self, position):
        pass

    async def on_camera(self, fix):
        pass

    async def on_status(self, status):
        # wait until our motion ceases before calling task_done()
        if self.stopping:
            if status.stopped():
                await self.app.task_done()
            return

    async def stop(self):
        print(f'[{time.time()}] Align.stop()', flush=True)
        if self.should_stop:
            await self.app.controller.set_throttle_pwm(0)
            self.stopping = True
        else:
            await self.app.task_done()

class ContactCone(Task):
    def __new__(cls, *args, **kwargs):
        return super().__new__(cls)

    def __init__(self, app):
        self.app = app
        self.stopping = False

    async def on_enter(self):
        print(f'[{time.time()}] ContactCone.on_enter()', flush=True)

    async def on_exit(self):
        print(f'[{time.time()}] ContactCone.on_exit()', flush=True)

    async def on_imu(self, orientation, heading):
        pass

    async def on_gps(self, position):
        pass

    async def on_camera(self, fix):
        if self.stopping:
            return

        if fix is None:
            print(f'[{time.time()}] ContactCone.on_camera(): lost fix, aborting')
            await self.stop()
            return

        error = fix.x - 640
        print(f'[{time.time()}] ContactCone.on_camera(): centering error = {error}')
        await self.app.controller.set_steering(-error * 2)

    async def on_status(self, status):
        # wait until our motion ceases before calling task_done()
        if self.stopping:
            if status.stopped():
                await self.app.task_done()
            return

        # if the bumper is pressed, we contacted the cone!
        if status.bumperPressed != 0:
            print(f'[{time.time()}] ContactCone.on_status(): Cone Contacted', flush=True)
            await self.stop()
        else:
            await self.app.controller.crawl()

    async def stop(self):
        print(f'[{time.time()}] ContactCone.stop()', flush=True)
        await self.app.controller.set_throttle_pwm(0)
        self.stopping = True

class App():
    def __new__(cls, *args, **kwargs):
        return super().__new__(cls)

    async def imu_task(self):
        # get one IMU measurement to know our starting heading
        orientation = await self.imu.get_orientation()
        offset = orientation.get_heading()

        while not self.completed:
            orientation = await self.imu.get_orientation()
            heading = normalize_heading(orientation.get_heading() - offset)
            print(f'[{time.time()}] heading = {heading}', flush=True)

            if self.active_task is not None:
                await self.active_task.on_imu(orientation, heading)

    async def gps_task(self):
        while not self.completed:
            position = await self.gps.get_position()
            print(f'[{time.time()}] gps = {position}', flush=True)

            if self.active_task is not None:
                await self.active_task.on_gps(position)

    async def camera_task(self):
        while not self.completed:
            fix = await self.camera.get_fix()
            print(f'[{time.time()}] fix = {fix}', flush=True)

            if self.active_task is not None:
                await self.active_task.on_camera(fix)

    async def task_done(self):
        # move to the next task if we have one
        await self.active_task.on_exit()
        self.tid = self.tid + 1
        if self.tid >= len(self.tasks):
            self.completed = True
        else:
           self.active_task = self.tasks[self.tid]
           await self.active_task.on_enter()

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

        # start first task
        print(f'Starting Run', flush=True)
        self.active_task = self.tasks[self.tid]
        await self.active_task.on_enter()

        # status loop
        while not self.completed:
            print(f'[{time.time()}] please arm to continue')
            await self.controller.wait_until_armable()
            await self.controller.arm()
            self.active = True

            # status loop
            while True:
                status = await self.controller.get_status()
                print(f'[{time.time()}] status = {status}')

                if not status.armed():
                    # we were disarmed (remotely, hopefully)
                    print(f'[{time.time()}] disarmed, waiting for idle')
                    self.active = False
                    await self.controller.wait_for_idle()
                    break
                elif self.completed:
                    # finished tasks
                    break
                elif self.active_task is not None:
                    await self.active_task.on_status(status)

        # stop robot
        await self.controller.set_throttle_pwm(0)
        await asyncio.sleep(1)

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

    def __init__(self):
        self.controller = MotionController.Connection()
        self.gps = GPS.Connection()
        self.imu = IMU.Connection()
        self.camera = Camera.Connection()

        # tasks to perform for this run
        self.completed = False
        self.active = False
        self.active_task = None
        self.tid = 0
        self.tasks = [\
            # Go to forward left of cone
            #Align(self, heading = -30, velocity = -0.5),
            #Drive(self, heading = -30, velocity = 0.5, distance = 1.0),
            #Align(self, heading = 0, velocity = 0.5),
            
            # Go to forward right of cone
            #Align(self, heading = 30, velocity = -0.5),
            #Drive(self, heading = 30, velocity = 0.5, distance = 1.0),
            #Align(self, heading = 0, velocity = 0.5),
            
            # Boop Cone
            ContactCone(self),
            Drive(self, heading = 0, velocity = -0.5, distance = -0.5),

            # Multi-point turn,
            Align(self, heading = -30, velocity = -0.5),
            Align(self, heading = -60, velocity = 0.5),
            Align(self, heading = -90, velocity = -0.5),
            Align(self, heading = -120, velocity = 0.5),
            Align(self, heading = -150, velocity = -0.5),
            Align(self, heading = -180, velocity = 0.5),
    
            # Go to forward right of cone
            #Align(self, heading = 30, velocity = -0.5),
            #Drive(self, heading = 30, velocity = 0.5, distance = 1.0),
            #Align(self, heading = 0, velocity = 0.5),
        ]

# run asynchronous app
if __name__ == "__main__":
    async def main():
        app = App()
        await app.run()

    asyncio.run(main())
