#!/usr/bin/env python
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

import asyncio
import math
import time

from kybernetes import MotionController, normalize_heading

class App():
    def __new__(cls, *args, **kwargs):
        return super().__new__(cls)

    def __init__(self, reset_dhi = False):
        self.controller = MotionController.Connection()
        self.suppress_orientation = False
        self.reset_dhi = reset_dhi

    async def orientation_task(self):
        while True:
            q = await self.controller.get_orientation()

            # IMU must be ready, reset DHI if needed
            if self.reset_dhi:
                await self.controller.reset_dhi_corrector()
                self.reset_dhi = False
                print(f'resetting dhi corrector...')

            # Display orientation if not suppressed
            heading = math.atan2(2.0 * (q.x*q.y - q.w*q.z), q.w*q.w - q.x*q.x + q.y*q.y - q.z*q.z) * 57.2957795
            heading = normalize_heading(heading)
            pitch = math.asin(2.0 * (q.y*q.z + q.w*q.x)) * 57.2957795
            roll = math.atan2(2.0 * (q.w*q.y - q.x*q.z), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z) * 57.2957795
            if not self.suppress_orientation:
                print(f'[{time.time()}] heading = {heading}, roll = {roll}, pitch = {pitch}')

    async def run(self):
        await self.controller.start()

        # start a task to receive the orientation
        self.imu_task = asyncio.create_task(self.orientation_task())
        while True:
            s = await self.controller.get_status()
            if s.imuStatus & 0x80:
                self.suppress_orientation = False
            elif s.imuStatus != 0:
                print(f'[{time.time()}] DHI Corrector: Invalid')
                self.suppress_orientation = True

# run asynchronous app
if __name__ == "__main__":
    async def main():
        await App(reset_dhi=False).run()

    asyncio.run(main())
