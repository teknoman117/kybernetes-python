#!/usr/bin/env python
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

import asyncio

from kybernetes import MotionController

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
            if not self.suppress_orientation:
                print(f'orientation = {q}')

    async def run(self):
        await self.controller.start()

        # start a task to receive the orientation
        self.imu_task = asyncio.create_task(self.orientation_task())
        while True:
            s = await self.controller.get_status()
            if s.imuStatus & 0x80:
                print(f'status = {s}')
                self.suppress_orientation = False
            else:
                print(f'DHI Corrector: Invalid - {bin(s.imuStatus)}')
                self.suppress_orientation = True

# run asynchronous app
if __name__ == "__main__":
    async def main():
        await App(reset_dhi=False).run()

    asyncio.run(main())
