#!/usr/bin/env python

import asyncio

from kybernetes import MotionController

async def main(loop):
    direction = False
    directionWasSet = False
    
    # listen to the IMU
    
    
    # open motion controller connection
    controller = MotionController.Connection()
    await controller.open()

    # send the configuration packet
    controller.sendPacket(MotionController.ConfigurationPacket(\
        servoThrottleDeadzoneForward = 1525,
        servoThrottleDeadzoneBackward = 1475,
        servoSteeringDeadzoneLeft = 1580,
        servoSteeringDeadzoneRight = 1580,
        Kp = 0.15,
        Ki = 0.25,
        Kd = 0.01
    ))

    # packet reactor
    while True:
        packet = await controller.receivePacket()
        
        if type(packet) is MotionController.StatusPacket:
            # arm the system if possible
            if packet.remote.state == MotionController.KILL_SWITCH_STATE_DISARMED and packet.remote.armable:
                controller.sendPacket(MotionController.SendArmPacket())
                
                if not directionWasSet:
                    directionWasSet = True
                    direction = not direction
            
            # otherwise issue steering adjustments
            elif packet.remote.state == MotionController.KILL_SWITCH_STATE_ARMED:
                directionWasSet = False
                steeringInput = -(packet.remote.servoSteeringInput - 1500)
                if direction:
                    target = 50
                else:
                    target = -50

                controller.sendPacket(MotionController.SendKeepalivePacket())
                controller.sendPacket(MotionController.SteeringSetPacket(servoSteeringInput=steeringInput))
                controller.sendPacket(MotionController.ThrottleSetPIDPacket(servoThrottlePIDTarget=target))
        else:
            print(f'non-status packet: {packet}')

        # flush all writes
        await controller.drain()

# run loop
loop = asyncio.get_event_loop()
loop.run_until_complete(main(loop))
loop.close()
