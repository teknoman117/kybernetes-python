from asyncio import get_event_loop, create_task, wait_for, Event
from crc8 import crc8
from ctypes import c_uint8, c_uint16, c_int16, c_float, memmove, pointer, sizeof, Structure
from serial_asyncio import open_serial_connection

import time

DEFAULT_BAUDRATE = 1000000
DEFAULT_DEVICE = "/dev/motioncontroller"

# Nack packet error types
FAILURE_TYPE_INVALID_STATE_TRANSITION = 0xC0
FAILURE_TYPE_REMOTELY_DISABLED = 0xD0
FAILURE_TYPE_INVALID_WHEN_ARMED = 0xE0
FAILURE_TYPE_INVALID_WHEN_DISARMED = 0xE1
FAILURE_TYPE_BAD_CHECKSUM = 0xFF

# Motion controller packet types
PACKET_TYPE_CONFIGURATION_SET = 0x10
PACKET_TYPE_CONFIGURATION_GET = 0x11
PACKET_TYPE_STEERING_SET = 0x20
PACKET_TYPE_THROTTLE_SET_PWM = 0x30
PACKET_TYPE_THROTTLE_SET_PID = 0x40
PACKET_TYPE_SEND_ARM = 0xA0
PACKET_TYPE_SEND_KEEPALIVE = 0xA1
PACKET_TYPE_SEND_DISARM = 0xA2
PACKET_TYPE_STATUS = 0xD0
PACKET_TYPE_NACK = 0xE0
PACKET_TYPE_SYNC = 0xFF

# Motion controller states
STATE_DISABLED = 0x00
STATE_DISABLING = 0x10
STATE_STOPPED = 0x20
STATE_MOVING_FORWARD_PWM = 0x30
STATE_MOVING_FORWARD_PID = 0x31
STATE_MOVING_BACKWARD_PWM = 0x40
STATE_MOVING_BACKWARD_PID = 0x41

# Motion controller (kill controller) states
KILL_SWITCH_STATE_DISARMED = 0
KILL_SWITCH_STATE_DISARMING_REQUESTED = 1
KILL_SWITCH_STATE_DISARMING_REMOTE = 2
KILL_SWITCH_STATE_DISARMING_SOFTWARE_TIMEOUT = 3
KILL_SWITCH_STATE_ARMED = 4

class PIDFrame(Structure):
    _pack_ = 1
    _fields_ = [
        ("Input", c_int16 * 3),
        ("Target", c_int16),
        ("Output", c_float),
        ("e", c_int16),
        ("pTerm", c_int16),
        ("iTerm", c_float),
        ("dTerm", c_float),
        ("Enabled", c_uint8)
    ]

    def __format__(self, spec):
        return f'''PIDFrame {{
            Enabled={self.Enabled},
            Input=({self.Input[0]}, {self.Input[1]}, {self.Input[2]}),
            Target={self.Target},
            Output={self.Output},
            e={self.e},
            pTerm={self.pTerm},
            iTerm={self.iTerm},
            dTerm={self.dTerm}
        }}'''

class KillSwitchStatusPacket(Structure):
    _pack_ = 1
    _fields_ = [
        ("servoSteeringInputUpdated", c_uint8, 1),
        ("servoThrottleInputUpdated", c_uint8, 1),
        ("armable", c_uint8, 1),
        ("unused1", c_uint8, 2),
        ("state", c_uint8, 3),
        ("servoSteeringInput", c_uint16),
        ("servoThrottleInput", c_uint16),
    ]

    def __format__(self, spec):
        return f'''KillSwitchStatusPacket {{
            servoSteeringInputUpdated={self.servoSteeringInputUpdated},
            servoThrottleInputUpdated={self.servoThrottleInputUpdated},
            armable={self.armable},
            state={self.state},
            servoSteeringInput={self.servoSteeringInput},
            servoThrottleInput={self.servoThrottleInput},
        }}'''

class StatusPacket(Structure):
    RECEIVE_TYPE = PACKET_TYPE_STATUS

    _pack_ = 1
    _fields_ = [
        ("state", c_uint8),
        ("remote", KillSwitchStatusPacket),
        ("batteryLow", c_uint8),
        ("align1", c_uint8),
        ("motion", PIDFrame),
    ]

    def __format__(self, spec):
        return f'''StatusPacket {{
            state={self.state},
            remote={self.remote},
            batteryLow={self.batteryLow},
            motion={self.motion}
        }}'''

class ConfigurationPacket(Structure):
    RECEIVE_TYPE = PACKET_TYPE_CONFIGURATION_GET
    SEND_TYPE = PACKET_TYPE_CONFIGURATION_SET

    _pack_ = 1
    _fields_ = [
        ("servoSteeringDeadzoneLeft", c_uint16),
        ("servoSteeringDeadzoneRight", c_uint16),
        ("servoThrottleDeadzoneForward", c_uint16),
        ("servoThrottleDeadzoneBackward", c_uint16),
        ("Kp", c_float),
        ("Ki", c_float),
        ("Kd", c_float)
    ]

    def __format__(self, spec):
        return f'''ConfigurationPacket {{
            servoSteeringDeadzoneLeft={self.servoSteeringDeadzoneLeft},
            servoSteeringDeadzoneRight={self.servoSteeringDeadzoneRight},
            servoThrottleDeadzoneForward={self.servoThrottleDeadzoneForward},
            servoThrottleDeadzoneBackward={self.servoThrottleDeadzoneBackward},
            Kp={self.Kp},
            Ki={self.Ki},
            Kd={self.Kd}
        }}'''

class SteeringSetPacket(Structure):
    SEND_TYPE = PACKET_TYPE_STEERING_SET

    _pack_ = 1
    _fields_ = [
        ("position", c_int16)
    ]

    def __format__(self, spec):
        return f'SteeringSetPacket(position={self.position})'

class ThrottleSetPWMPacket(Structure):
    SEND_TYPE = PACKET_TYPE_THROTTLE_SET_PWM

    _pack_ = 1
    _fields_ = [
        ("value", c_int16)
    ]

    def __format__(self, spec):
        return f'ThrottleSetPWMPacket(value={self.value})'

class ThrottleSetPIDPacket(Structure):
    SEND_TYPE = PACKET_TYPE_THROTTLE_SET_PID

    _pack_ = 1
    _fields_ = [
        ("target", c_int16)
    ]

    def __format__(self, spec):
        return f'ThrottleSetPIDPacket(target={self.target})'

class SendArmPacket(Structure):
    SEND_TYPE = PACKET_TYPE_SEND_ARM
    _pack_ = 1
    _fields_ = []
    def __format__(self, spec):
        return 'SendArmPacket()'

class SendKeepalivePacket(Structure):
    SEND_TYPE = PACKET_TYPE_SEND_KEEPALIVE
    _pack_ = 1
    _fields_ = []
    def __format__(self, spec):
        return 'SendKeepalivePacket()'

class SendDisarmPacket(Structure):
    SEND_TYPE = PACKET_TYPE_SEND_DISARM
    _pack_ = 1
    _fields_ = []
    def __format__(self, spec):
        return 'SendDisarmPacket()'

class ConfigurationGetCommand(Structure):
    SEND_TYPE = PACKET_TYPE_CONFIGURATION_GET
    _pack_ = 1
    _fields_ = []
    def __format__(self, spec):
        return 'ConfigurationGetCommand()'

class SyncPacket(Structure):
    SEND_TYPE = PACKET_TYPE_SYNC
    RECEIVE_TYPE = PACKET_TYPE_SYNC

    _pack_ = 1
    _fields_ = [
        ("bytes", c_uint8 * 15)
    ]

class AckPacket(Structure):
    _pack_ = 1
    _fields_ = []

class ConfigurationSetAckPacket(AckPacket):
    def __format__(self, spec):
        return f"ConfigurationSetAckPacket {{}}"

class SteeringSetAckPacket(AckPacket):
    def __format__(self, spec):
        return f"SteeringSetAckPacket {{}}"

class ThrottleSetPWMAckPacket(AckPacket):
    def __format__(self, spec):
        return f"ThrottleSetPWMAckPacket {{}}"

class ThrottleSetPIDAckPacket(AckPacket):
    def __format__(self, spec):
        return f"ThrottleSetPIDAckPacket {{}}"

class SendArmAckPacket(AckPacket):
    def __format__(self, spec):
        return f"SendArmAckPacket {{}}"

class SendKeepaliveAckPacket(AckPacket):
    def __format__(self, spec):
        return f"SendKeepaliveAckPacket {{}}"

class SendDisarmAckPacket(AckPacket):
    def __format__(self, spec):
        return f"SendDisarmAckPacket {{}}"

class NackPacket(Structure):
    _pack_ = 1
    _fields_ = [
        ("command_type_id", c_uint8),
        ("failure_type_id", c_uint8)
    ]

    def __format__(self, spec):
        return f'''NackPacket {{packetType={self.packetType}, failureType={self.failureType}}}'''

PACKET_TYPE_BY_ID = {
    PACKET_TYPE_CONFIGURATION_SET : ConfigurationSetAckPacket,
    PACKET_TYPE_CONFIGURATION_GET : ConfigurationPacket,
    PACKET_TYPE_STEERING_SET : SteeringSetAckPacket,
    PACKET_TYPE_THROTTLE_SET_PWM : ThrottleSetPWMAckPacket,
    PACKET_TYPE_THROTTLE_SET_PID : ThrottleSetPIDAckPacket,
    PACKET_TYPE_SEND_ARM : SendArmAckPacket,
    PACKET_TYPE_SEND_KEEPALIVE : SendKeepaliveAckPacket,
    PACKET_TYPE_SEND_DISARM : SendDisarmAckPacket,
    PACKET_TYPE_STATUS : StatusPacket,
    PACKET_TYPE_NACK : NackPacket,
    PACKET_TYPE_SYNC : SyncPacket
}

class InvalidChecksumError(Exception):
    pass

class DesyncError(Exception):
    pass

class Connection():
    def __new__(cls, *args, **kwargs):
        return super().__new__(cls)

    def __init__(self, device=DEFAULT_DEVICE, baudrate=DEFAULT_BAUDRATE):
        self.device = device
        self.baudrate = baudrate
        self.stop_requested = False
        self.command_futures = []
        self.status_futures = []
        self.status = StatusPacket()
        self.armed_event = Event()
        self.disarmed_event = Event()
        self.idle_event = Event()

    async def synchronize(self):
        while True:
            # find the first sync byte (0xff)
            data = await wait_for(self.reader.readexactly(1), timeout=5.0)
            if data[0] != 0xff:
                continue

            # sync packet is exactly sixteen 0xff's, so find the remaining 15
            candidate = await self.reader.readexactly(15)
            target = bytes([0xff]) * 15
            if candidate != target:
                # force a sync packet
                self.writer.write('\xff')
                await self.writer.drain()
            else:
                return

    def send_packet(self, packet):
        data = bytearray()
        data.append(packet.SEND_TYPE)
        if sizeof(packet) > 0:
            data.extend(packet)

        hash = crc8(data)
        data.extend(hash.digest())

        print(f'[DEBUG {time.time()}] send_packet: {packet}')
        self.writer.write(data)

    async def receive_packet(self):
        pTypeId = (await self.reader.readexactly(1))[0]
        pType = PACKET_TYPE_BY_ID[pTypeId]
        data = await self.reader.readexactly(sizeof(pType))
        checksumRemote = (await self.reader.readexactly(1))[0]

        hash = crc8(bytes([pTypeId]) + data)
        checksumLocal = hash.digest()[0]
        if checksumLocal != checksumRemote:
            raise InvalidChecksumError

        packet = pType()
        memmove(pointer(packet), data, sizeof(pType))
        print(f'[DEBUG {time.time()}] receive_packet: {packet}')
        return (pTypeId, packet)

    async def send_command(self, packet):
        # this only works because motion controller processes commands in order
        f = get_event_loop().create_future()
        self.command_futures.append((type(packet).SEND_TYPE, f))
        self.send_packet(packet)
        await f
        return f.result()

    async def get_status(self):
        f = get_event_loop().create_future()
        self.status_futures.append(f)
        return await f

    async def arm(self):
        await self.send_command(SendArmPacket())
        await self.armed_event.wait()
        create_task(self.keepalive())

    async def disarm(self):
        await self.send_command(SendDisarmPacket())
        await self.disarmed_event.wait()

    async def wait_until_armable(self):
        while True:
            s = await self.get_status()
            if s.remote.armable:
                break

    async def wait_for_disarm(self):
        await self.disarmed_event.wait()

    async def wait_for_idle(self):
        await self.idle_event.wait()

    # keepalive loop
    async def keepalive(self):
        while not self.disarmed_event.is_set():
            await self.send_command(SendKeepalivePacket())
            await self.get_status()

    # packet reactor
    async def loop(self):
        while not self.stop_requested:
            (packet_type_id, packet) = await self.receive_packet()

            if type(packet) is StatusPacket:
                # detect arming state transitions
                if packet.remote.state == KILL_SWITCH_STATE_ARMED:
                    self.armed_event.set()
                    self.disarmed_event.clear()
                    self.idle_event.clear()
                elif packet.remote.state == KILL_SWITCH_STATE_DISARMED:
                    self.armed_event.clear()
                    self.disarmed_event.set()
                    self.idle_event.set()
                else:
                    # one of the "disarming" states
                    self.armed_event.clear()
                    self.disarmed_event.set()
                    self.idle_event.clear()

                # wake up 'get_status()' callers
                for f in self.status_futures:
                    f.set_result(packet)
                self.status_futures.clear()

                # keep a copy of the packet
                self.status = packet

            elif type(packet) is SyncPacket:
                pass

            elif type(packet) is NackPacket:
                (command_type_id, f) = self.command_futures.pop(0)
                if packet.command_type_id != command_type_id:
                    # we have a de-sync between commands and responses
                    # TODO: tear down and re-establish motion controller link
                    raise DesyncError
                f.set_result(packet)

            else:
                (command_type_id, f) = self.command_futures.pop(0)
                if packet_type_id != command_type_id:
                    # we have a de-sync between commands and responses
                    # TODO: tear down and re-establish motion controller link
                    raise DesyncError
                f.set_result(packet)


    async def start(self):
        self.reader, self.writer = await open_serial_connection(url=self.device, baudrate=self.baudrate)
        await self.synchronize()
        self.task = create_task(self.loop())

    async def stop(self):
        self.stop_requested = True
        await self.task