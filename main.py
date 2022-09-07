#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

left =0
right =0

def mainFunc():
   

    ## Create your objects here.
    ev3 = EV3Brick()

    # Motors setup
    leftMotor=Motor(Port.A)
    righttMotor=Motor(Port.D)

    # Ultrasonic sensors setup
    frontUltrasonic = UltrasonicSensor(Port.S1)
    leftUltrasonic1 = UltrasonicSensor(Port.S2)
    leftUltrasonic2 = UltrasonicSensor(Port.S3)

    # Movesteering function
    moveSteering = DriveBase(leftMotor, righttMotor, 56, 120)
    moveSteering.settings(350, 350)

    # Gyroscope sensor
    gyroscope = GyroSensor(Port.S4, Direction.COUNTERCLOCKWISE)
    # gyroscope.speed(90)


    ## Create your functions here
    # Turn right 90 degree
    def right_90():
        moveSteering.turn(90)
        angle = gyroscope.angle()
        ev3.screen.print(angle)
        while(angle < 85):
            angle = gyroscope.angle()
            ev3.screen.print(angle)
            angle_error = 85-angle
            moveSteering.turn(angle_error)
        gyroscope.reset_angle(0)

        # global left
        # global right
        # left =0
        # right +=1
        # if (right == 2):
        #     right =0
        #     stop()

    # Turn left 90 degree
    def left_90():
        moveSteering.turn(-90)
        angle = gyroscope.angle()
        ev3.screen.print(angle)
        while(angle > -85):
            angle = gyroscope.angle()
            ev3.screen.print(angle)
            angle_error = -85-angle
            moveSteering.turn(angle_error)
        gyroscope.reset_angle(0)

        # global right
        # global left
        # right = 0
        # left +=1
        # ev3.screen.print(left)
        # if (left == 2):
        #     left =0
        #     stop()



    # Move forward without stopping
    def moveStraight(dist):

        # Sensors reading
        leftSensor1 = leftUltrasonic1.distance()
        leftSensor2 = leftUltrasonic2.distance()
        frontSensor = frontUltrasonic.distance()

        if ( leftSensor1 < 200 and leftSensor2 < 200):
            if( leftSensor1 < 85):
                moveSteering.drive(350,20)

            if( leftSensor1 > 85):
                moveSteering.drive(350,-20)
            
            # Calculating error
            error = leftSensor1 - leftSensor2 - 3

            if (error < -5  and error > 5):
                gyroscope.reset_angle(0)
            if ( leftSensor2 > 75):
                moveSteering.drive(350,error*-1)

            
        else:
                moveSteering.drive(350,0)


    # Stop function'
    def stop():
        moveSteering.stop()
    
    # Move forward 1 cell
    def moveOneCell(dist):
        while (dist % 360 != 0):
            moveSteering.drive(350, 0)
        # distance = moveSteering.reset()
    def moveHalfCell(dist):
        # distance = moveSteering.reset()
        distance = 1
        while (distance % 180 > 0):
            moveSteering.drive(350, 0)
            distance = moveSteering.distance()
        # distance = moveSteering.reset()

    



    # Write your program here.
    ev3.speaker.beep()

    # First step of the robot to locate at the center of the cell
    moveSteering.straight(65)
    distance = moveSteering.reset()

    while(1):
        
        
        angle = gyroscope.angle()
        ev3.screen.print(angle)
        # Sensors reading
        leftSensor1 = leftUltrasonic1.distance()
        leftSensor2 = leftUltrasonic2.distance()
        frontSensor = frontUltrasonic.distance()


        # Calculate moved distance
        distance = moveSteering.distance()
        

        # Test2
        
            
        if (leftSensor1 >250 and leftSensor2 >250  ):
                # moveHalfCell(distance)
                moveSteering.straight(60)
                left_90()
                # moveHalfCell(distance)
                moveSteering.straight(300)
                
        else:
            if(frontSensor > 125):
                moveStraight(distance)
            else:
                right_90()
                # moveOneCell()
        
        # End test2

       

        

mainFunc()