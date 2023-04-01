#!/usr/bin/env python

import asyncio
import time

from kybernetes import MotionController

async def remote_control():
    # open motion controller connection
    controller = MotionController.Connection()
    await controller.start()

    # send the configuration packet
    await controller.send_command(MotionController.ConfigurationPacket(\
        servoThrottleDeadzoneForward = 1525,
        servoThrottleDeadzoneBackward = 1475,
        servoSteeringDeadzoneLeft = 1580,
        servoSteeringDeadzoneRight = 1580,
        Kp = 0.15,
        Ki = 0.25,
        Kd = 0.01
    ))

    while True:
        # Forward motion
        print('arm controller to move forward')
        await controller.wait_until_armable()
        await controller.arm()
        print('moving forward')
        await controller.send_command(MotionController.ThrottleSetPIDPacket(target=50))
        while True:
            s = await controller.get_status()
            if s.remote.state != MotionController.KILL_SWITCH_STATE_ARMED:
                break
            position = -(s.remote.servoSteeringInput - 1500)
            await controller.send_command(MotionController.SteeringSetPacket(position=position))
        print('disarmed. waiting for idle.')
        await controller.wait_for_idle()

        # Backward motion
        print('arm controller to move backward')
        await controller.wait_until_armable()
        await controller.arm()
        print('moving backward')
        await controller.send_command(MotionController.ThrottleSetPIDPacket(target=-50))
        while True:
            s = await controller.get_status()
            if s.remote.state != MotionController.KILL_SWITCH_STATE_ARMED:
                break
            position = -(s.remote.servoSteeringInput - 1500)
            await controller.send_command(MotionController.SteeringSetPacket(position=position))
        print('disarmed. waiting for idle.')
        await controller.wait_for_idle()

# run main if we're being executed
if __name__ == "__main__":
    asyncio.run(remote_control())
