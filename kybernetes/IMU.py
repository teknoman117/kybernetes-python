# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

from asyncio import open_connection, open_unix_connection

from kybernetes import normalize_heading

import json
import math

POWER_OF_2_30 = 1073741824.0

class Quaternion():
    def __init__(self, w=1.0, x=0.0, y=0.0, z=0.0):
        self.w = w
        self.x = x
        self.y = y
        self.z = z

    def __format__(self, spec):
        if spec == 'euler':
            roll = self.roll()
            pitch = self.pitch()
            yaw = self.yaw()
            return f'Quaternion(roll={roll}, pitch={pitch}, yaw={yaw})'
        else:
            return f'Quaterion(w={self.w}, x={self.x}, y={self.y}, z={self.z})'

    def roll(self):
        sinr_cosp = 2.0 * (self.w * self.x + self.y * self.z)
        cosr_cosp = 1.0 - 2.0 * (self.x**2 + self.y**2)
        return math.atan2(sinr_cosp, cosr_cosp)

    def pitch(self):
        sinp = math.sqrt(1.0 + 2.0 * (self.w * self.y - self.x * self.z))
        cosp = math.sqrt(1.0 - 2.0 * (self.w * self.y - self.x * self.z))
        return 2.0 * math.atan2(sinp, cosp) - math.pi / 2.0

    def yaw(self):
        siny_cosp = 2.0 * (self.w * self.z + self.x * self.y)
        cosy_cosp = 1.0 - 2.0 * (self.y**2 + self.z**2)
        return math.atan2(siny_cosp, cosy_cosp)

class Orientation():
    def __new__(cls, *args, **kwargs):
        return super().__new__(cls)
    
    def __init__(self, rotation=Quaternion(), heading=None):
        self.rotation = rotation
        if heading is None:
            self.heading = math.degrees(rotation.yaw())
        else:
            self.heading = heading
    
    def get_rotation(self):
        return self.rotation

    def get_heading(self):
        return self.heading

# super basic gpsd client that just scrapes position data
class Connection():
    def __init__(self, device='/run/imud/imu.sock', host=None, port=4000):
        self.device = device
        self.host = host
        self.port = port

    async def start(self):
        if self.host is not None:
            self.reader, self.writer = await open_connection(self.host, port=self.port)
        elif self.device is not None:
            self.reader, self.writer = await open_unix_connection(self.device)

    async def stop(self):
        self.writer.close()
        self.reader = None
        self.writer = None

    async def get_orientation(self):
        data = await self.reader.readline()
        measurement = json.loads(data)

        # recompute orientation from imud
        #q = Quaternion()
        #raw = measurement['orientation_raw']
        #q.x = raw['x'] / POWER_OF_2_30
        #q.y = raw['y'] / POWER_OF_2_30
        #q.z = raw['z'] / POWER_OF_2_30
        #q.w = math.sqrt(1.0 - (q.x**2 + q.y**2 + q.z**2))
        #return Orientation(rotation=q)

        # use imud quaternion directly
        q = Quaternion()
        q.w = measurement['orientation']['w']
        q.x = measurement['orientation']['x']
        q.y = measurement['orientation']['y']
        q.z = measurement['orientation']['z']
        return Orientation(rotation=q, heading=normalize_heading(measurement['heading']))