#!/usr/bin/env python

import asyncio
import time
import math

from kybernetes import MotionController, GPS, IMU, Camera, normalize_heading

DRIVE_STEERING_KP = 50
DRIVE_THROTTLE_KP = 0.75
ALIGN_STEERING_KP = 100
ALIGN_THROTTLE_KP = 1.0 / 10.0
MOVETO_STEERING_KP = 25

MINIMUM_VELOCITY = 0.3

def clamp(value, minimum, maximum):
    return max(min(value, maximum), minimum)

class Task():
    pass

# Task of tasks
class TaskGroup(Task):
    def __new__(cls, *args, **kwargs):
        return super().__new__(cls)

    def __init__(self, app, done):
        self.app = app
        self.done = done

        self.active_task = None
        self.tid = 0
        self.tasks = []

    async def task_done(self):
        await self.active_task.on_exit()
        self.tid = self.tid + 1
        if self.tid >= len(self.tasks):
            await self.done()
        else:
            self.active_task = self.tasks[self.tid]
            await self.active_task.on_enter()

    async def on_enter(self):
        self.active_task = self.tasks[self.tid]
        await self.active_task.on_enter()

    async def on_exit(self):
        pass

    async def on_imu(self, orientation, heading):
        if self.active_task is not None:
            await self.active_task.on_imu(orientation, heading)

    async def on_gps(self, position):
        if self.active_task is not None:
            await self.active_task.on_gps(position)

    async def on_camera(self, fix):
        if self.active_task is not None:
            await self.active_task.on_camera(fix)

    async def on_status(self, status):
        if self.active_task is not None:
            await self.active_task.on_status(status)

class Drive(Task):
    def __new__(cls, *args, **kwargs):
        return super().__new__(cls)

    def __init__(self, app, done, distance, heading, velocity):
        self.app = app
        self.done = done
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

        # wait until our motion ceases before calling done()
        if self.stopping:
            if status.stopped():
                await self.done()
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

    def __init__(self, app, done, heading, velocity, should_stop=True):
        self.app = app
        self.done = done
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
                await self.done()
            return

    async def stop(self):
        print(f'[{time.time()}] Align.stop()', flush=True)
        if self.should_stop:
            await self.app.controller.set_throttle_pwm(0)
            self.stopping = True
        else:
            await self.done()

class ContactCone(Task):
    def __new__(cls, *args, **kwargs):
        return super().__new__(cls)

    def __init__(self, app, done):
        self.app = app
        self.done = done
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
                await self.done()
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
        
class FindCone(Task):
    def __new__(cls, *args, **kwargs):
        return super().__new__(cls)

    def __init__(self, app, done):
        self.app = app
        self.done = done
        self.cumulative_heading = 0
        self.previous_heading = None
        self.stopping = False

    async def on_enter(self):
        print(f'[{time.time()}] FindCone.on_enter()', flush=True)

    async def on_exit(self):
        print(f'[{time.time()}] FindCone.on_exit()', flush=True)

    async def on_imu(self, orientation, heading):
        if self.stopping:
            return

        # figure out how much we've turned in the last cycle
        if self.previous_heading is None:
            self.previous_heading = heading
        delta = normalize_heading(heading - self.previous_heading)
        self.cumulative_heading = self.cumulative_heading + delta
        self.previous_heading = heading
        
        print(f'[{time.time()}] FindCone.on_imu(): total rotation = {self.cumulative_heading}', flush=True)
        
        # abort on full rotation
        if abs(self.cumulative_heading) >= 360:
            print(f'[{time.time()}] FindCone.on_imu(): Failed to acquired fix', flush=True)
            await self.stop()

    async def on_gps(self, position):
        pass

    async def on_camera(self, fix):
        if self.stopping:
            return

        if fix is not None:
            print(f'[{time.time()}] FindCone.on_camera(): Acquired fix', flush=True)
            await self.stop()
            return

    async def on_status(self, status):
        # wait until our motion ceases before calling task_done()
        if self.stopping:
            if status.stopped():
                await self.done()
            return

        await self.app.controller.set_steering(500)
        await self.app.controller.set_throttle_pid(0.5)

    async def stop(self):
        print(f'[{time.time()}] FindCone.stop()', flush=True)
        r = await self.app.controller.set_throttle_pid(0)
        print(f'[{time.time()}] FindCone.stop() controller response = {r}', flush=True)
        #self.stopping = True
        await self.done()

class MoveTo(Task):
    def __new__(cls, *args, **kwargs):
        return super().__new__(cls)

    def __init__(self, app, done, target, velocity, should_stop=True):
        self.app = app
        self.done = done
        self.target = target
        self.velocity = velocity
        self.should_stop = should_stop
        self.heading_to_target = 0
        self.stopping = False

    async def on_enter(self):
        print(f'[{time.time()}] MoveTo.on_enter()', flush=True)

    async def on_exit(self):
        print(f'[{time.time()}] MoveTo.on_exit()', flush=True)

    async def on_imu(self, orientation, heading):
        if self.stopping:
            return

        error = -normalize_heading(self.heading_to_target - heading)
        response = int(MOVETO_STEERING_KP * error)
        print(f'[{time.time()}] MoveTo.on_imu(): heading error = {error}')
        await self.app.controller.set_steering(response)

    async def on_gps(self, position):
        if self.stopping:
            return

        self.heading_to_target = position.get_heading_to(self.target, magnetic=False)
        distance = position.get_distance_to(self.target)
        print(f'[{time.time()}] MoveTo.on_gps(): heading to target = {self.heading_to_target}, distance to target = {distance}')

        # slow down as we approach the target
        if distance > 7 or (not self.should_stop and distance > 1):
            speed = clamp(self.velocity, MINIMUM_VELOCITY, self.velocity)
            await self.app.controller.set_throttle_pid(speed)
        elif distance > 1:
            speed = clamp(self.velocity/4, MINIMUM_VELOCITY, self.velocity)
            await self.app.controller.set_throttle_pid(speed)
        else:
            await self.stop()

    async def on_camera(self, fix):
        pass

    async def on_status(self, status):
        # wait until our motion ceases before calling task_done()
        if self.stopping:
            if status.stopped():
                await self.done()
            return

    async def stop(self):
        print(f'[{time.time()}] MoveTo.stop()', flush=True)
        if self.should_stop:
            await self.app.controller.set_throttle_pwm(0)
            self.stopping = True
        else:
            await self.done()

# Task group to perform tasks relative to starting heading
class RelativeTaskGroup(TaskGroup):
    def __new__(cls, *args, **kwargs):
        return super().__new__(cls)

    def __init__(self, app, done):
        super().__init__(app, done)
        self.offset = None

    # override imu function so that all turns are relative to the starting heading
    async def on_imu(self, orientation, heading):
        if self.offset is None:
            self.offset = heading

        # TODO: rotate orientation as well
        heading = normalize_heading(heading - self.offset)
        await super().on_imu(orientation, heading)

# Perform a 6-point turn to rotate the robot around in a minimal amount of space
class TurnAround(RelativeTaskGroup):
    def __new__(cls, *args, **kwargs):
        return super().__new__(cls)

    def __init__(self, app, done):
        super().__init__(app, done)
        self.tasks = [
            Drive(app, self.task_done, heading = 0, velocity = -1.0, distance = -0.5),
            Align(app, self.task_done, heading = -30, velocity = -0.5),
            Align(app, self.task_done, heading = -60, velocity = 0.5),
            Align(app, self.task_done, heading = -90, velocity = -0.5),
            Align(app, self.task_done, heading = -120, velocity = 0.5),
            Align(app, self.task_done, heading = -150, velocity = -0.5),
            Align(app, self.task_done, heading = -180, velocity = 0.5),
        ]

# Move to the left of the cone
class MovePastLeft(RelativeTaskGroup):
    def __new__(cls, *args, **kwargs):
        return super().__new__(cls)

    def __init__(self, app, done):
        super().__init__(app, done)
        self.tasks = [
            Drive(app, self.task_done, heading = 0, velocity = -1.0, distance = -0.5),
            Align(app, self.task_done, heading = -30, velocity = -0.5),
            Drive(app, self.task_done, heading = -30, velocity = 0.5, distance = 1.5),
            Align(app, self.task_done, heading = 0, velocity = 0.5),
        ]

# Move to the left of the cone
class MovePastRight(RelativeTaskGroup):
    def __new__(cls, *args, **kwargs):
        return super().__new__(cls)

    def __init__(self, app, done):
        super().__init__(app, done)
        self.tasks = [
            Drive(app, self.task_done, heading = 0, velocity = -1.0, distance = -0.5),
            Align(app, self.task_done, heading = 30, velocity = -0.5),
            Drive(app, self.task_done, heading = 30, velocity = 0.5, distance = 1.5),
            Align(app, self.task_done, heading = 0, velocity = 0.5),
        ]

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
        await self.controller.disarm()

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
            MoveTo(self, self.task_done, target = GPS.Position(latitude=37.76968, longitude=-122.48330), velocity = 3),
            MoveTo(self, self.task_done, target = GPS.Position(latitude=37.76965, longitude=-122.48363).offset(heading=135, distance=4), velocity = 3, should_stop=True),
            FindCone(self, self.task_done),
            ContactCone(self, self.task_done),
            #TurnAround(self, self.task_done),
            Align(self, self.task_done, heading = 90, velocity = -1.0),
            MoveTo(self, self.task_done, target = GPS.Position(latitude=37.76968, longitude=-122.48330), velocity = 3, should_stop=False),
            MoveTo(self, self.task_done, target = GPS.Position(latitude=37.76970, longitude=-122.48288).offset(heading=-90, distance=4), velocity = 3, should_stop=True),
            FindCone(self, self.task_done),
            ContactCone(self, self.task_done),
            #TurnAround(self, self.task_done),
            Align(self, self.task_done, heading = -90, velocity = -1.0),
            MoveTo(self, self.task_done, target = GPS.Position(latitude=37.76968, longitude=-122.48330), velocity = 3, should_stop=False),
            MoveTo(self, self.task_done, target = GPS.Position(latitude=37.76965, longitude=-122.48363).offset(heading=135, distance=4), velocity = 3, should_stop=True),
            FindCone(self, self.task_done),
            ContactCone(self, self.task_done),
            #TurnAround(self, self.task_done),
            Align(self, self.task_done, heading = 90, velocity = -1.0),
            MoveTo(self, self.task_done, target = GPS.Position(latitude=37.76968, longitude=-122.48330), velocity = 3),
            MoveTo(self, self.task_done, target = GPS.Position(latitude=37.76968, longitude=-122.48330).offset(heading=180, distance=15), velocity = 3, should_stop=True),
        ]

# run asynchronous app
if __name__ == "__main__":
    async def main():
        app = App()
        await app.run()

    asyncio.run(main())
