#!/usr/bin/env python3
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

import asyncio

from kybernetes import Camera, MotionController

class App():
    def __new__(cls, *args, **kwargs):
        return super().__new__(cls)
    
    def __init__(self):
        self.controller = MotionController.Connection()
        self.camera = Camera.Connection()
        self.completed = False
        
    async def camera_task(self):
        while not self.completed:
            fix = await self.camera.get_fix()
            if fix is None:
                continue
            
            # update error
            error = -fix.x
            print(f'error = {error} (degrees)')
            await self.controller.set_steering(error * 32)
        
    async def run(self):
        await self.controller.start()
        await self.camera.start()
        
        camera_task = asyncio.create_task(self.camera_task())
        
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
            while True:
                s = await self.controller.get_status()
                if not s.armed():
                    # we were disarmed (remotely, hopefully)
                    await self.controller.wait_for_idle()
                    break
                elif s.bumperPressed != 0:
                    # we contacted cone
                    print('cone contacted')
                    self.completed = True
                    break
                else:
                    # otherwise update speed
                    await self.controller.crawl()

        # wait for sensor loops to finish
        await camera_task
        await self.controller.disarm()
        
        # tear down connections
        await self.camera.stop()
        await self.controller.stop()

# main
if __name__ == "__main__":
    async def main():
        await App().run()

    asyncio.run(main())
