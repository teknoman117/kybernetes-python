import asyncio
import json
import sys

class Fix():
    def __new__(cls, *args, **kwargs):
        return super().__new__(cls)
    
    def __init__(self, area=0, x=0, y=0):
        self.area = area
        self.x = x
        self.y = y
        pass
    
    def __format__(self, spec):
        return f'Fix(area={self.area}, x={self.x}, y={self.y})'

class Connection():
    def __new__(cls, *args, **kwargs):
        return super().__new__(cls)
    
    def __init__(self):
        pass
        
    async def start(self):
        self.proc = await asyncio.create_subprocess_exec(sys.executable, 'vision_process.py', stdout=asyncio.subprocess.PIPE, stdin=asyncio.subprocess.PIPE)
        await self.proc.stdout.readline()
    
    async def stop(self):
        self.proc.stdin.write(b'\n')
        await self.proc.stdin.drain()
        await self.proc.wait()

    async def get_fix(self):
        data = await self.proc.stdout.readline()
        ascii = data.decode('ascii').rstrip()
        fix = json.loads(ascii)
        if fix['found']:
            return Fix(area=fix['area'], x=fix['x'], y=fix['y'])
        else:
            return None