from asyncio import open_connection

from kybernetes import normalize_heading

import json
import math

DEFAULT_DEVICE = "/dev/gps"

class Position():
    EARTH = 6371009.0

    def __init__(self, latitude=0.0, longitude=0.0, declination=0.0):
        self.latitude = latitude
        self.longitude = longitude
        self.declination = declination
        self.epx = 0.0
        self.epy = 0.0
    
    def __format__(self, spec):
        return f'Position(latitude={self.latitude}, longitude={self.longitude}, declination={self.declination})'
    
    def get_distance_to(self, position, R = EARTH):
        phi1 = math.radians(self.latitude)
        phi2 = math.radians(position.latitude)
        dPhi_2 = (phi2 - phi1) / 2.0
        dLambda_2 = math.radians(position.longitude - self.longitude) / 2.0

        a = math.sin(dPhi_2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(dLambda_2)**2
        return R * 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0-a))

    def get_heading_to(self, position, magnetic=False):
        phi1 = math.radians(self.latitude)
        phi2 = math.radians(position.latitude)
        dLambda = math.radians(position.longitude - self.longitude)
        
        y = math.sin(dLambda) * math.cos(phi2)
        x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dLambda)
        theta = math.atan2(y,x)
        d = math.degrees(theta)
        if magnetic:
            # adjust for magnetic delination
            d = normalize_heading(d - self.declination)
        return d

    def offset(self, heading, distance, R = EARTH):
        def mod(y, x):
            return y - x * math.floor(y/x)

        lat1 = math.radians(self.latitude)
        lon1 = math.radians(self.longitude)
        tc = -math.radians(heading)
        d = distance / R

        lat = math.asin(math.sin(lat1) * math.cos(d) + math.cos(lat1) * math.sin(d) * math.cos(tc))
        dlon = math.atan2(math.sin(tc) * math.sin(d) * math.cos(lat1), math.cos(d) - math.sin(lat1) * math.sin(lat))
        lon = mod(lon1 - dlon + math.pi, 2*math.pi) - math.pi

        return Position(latitude = math.degrees(lat), longitude = math.degrees(lon))

    def get_latitude(self):
        return self.latitude
    
    def get_longitude(self):
        return self.longitude
    
    def get_declination(self):
        return self.declination

    def get_error(self):
        return self.epx, self.epy

# super basic gpsd client that just scrapes position data
class Connection():
    def __init__(self, device=DEFAULT_DEVICE, host='localhost', port=2947):
        self.device = device
        self.host = host
        self.port = port
        
    async def start(self):
        self.reader, self.writer = await open_connection(self.host, port=self.port)
        self.writer.write(b'?WATCH={"enable":true,"json":true};')
        await self.writer.drain()

    async def stop(self):
        self.writer.close()
        self.reader = None
        self.writer = None
    
    async def get_position(self):
        while True:
            data = await self.reader.readline()
            message = json.loads(data)
            if not 'class' in message:
                # skip non-gps messages
                continue
            elif message['class'] != 'TPV':
                # skip messages that aren't "Time Position Velocity" measurements
                continue
            elif not 'lat' in message or not 'lon' in message or not 'magvar' in message:
                # skip malformed TPV messages
                continue

            p = Position(latitude=message['lat'], longitude=message['lon'], declination=message['magvar'])
            if 'epx' in message and 'epy' in message:
                p.epx = message["epx"]
                p.epy = message["epy"]
            return p
