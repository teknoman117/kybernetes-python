#!/usr/bin/env python
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

import asyncio

from kybernetes import MotionController

class App():
    def __new__(cls, *args, **kwargs):
        return super().__new__(cls)
    
    def __init__(self):
        self.controller = MotionController.Connection()
    
    async def run(self):
        await self.controller.start()

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

        while True:
            # Forward motion
            print('arm controller to move forward')
            await self.controller.wait_until_armable()
            await self.controller.arm()
            print('moving forward')
            await self.controller.set_throttle_pid(0.5)
            while True:
                s = await self.controller.get_status()
                if not s.armed():
                    break
                await self.controller.set_steering(-s.remote.servoSteeringInput + 1500)
            print('disarmed. waiting for idle.')
            await self.controller.wait_for_idle()

            # Backward motion
            print('arm self.controller to move backward')
            await self.controller.wait_until_armable()
            await self.controller.arm()
            print('moving backward')
            await self.controller.set_throttle_pid(-0.5)
            while True:
                s = await self.controller.get_status()
                if not s.armed():
                    break
                await self.controller.set_steering(-s.remote.servoSteeringInput + 1500)
            print('disarmed. waiting for idle.')
            await self.controller.wait_for_idle()

# run asynchronous app
if __name__ == "__main__":
    async def main():
        await App().run()

    asyncio.run(main())
