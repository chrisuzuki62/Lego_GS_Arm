/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Written by Chris Suzuki
 * This file will drive the entire Interactive Lego Girl Scout Robot Arm
 *
 * The code REQUIRES that you DO have encoders on the motors that are 28 tick
 * and gear boxes attached as follows l_arm:3,4,5  u_arm:4,5,5  s_arm:4,5,5
 *
 *   The motors, servos, sensors should be configured as the following
 *   Motors
 *   Port 0 l_arm      *The Lower Arm of the Robot Arm
 *   Port 1 u_arm      *The Upper Arm of the Robot Arm
 *   Port 2 s_arm      *The Shoulder of the Robot Arm
 *
 *   Servos
 *   Port 0 hand       *Servo that controls the thumb and pinky movement
 *
 *   Sensors(digital)
 *   Port 0 touch
 *   Port 2 headin
 *   Port 4 headout
 *
 *   Sensors(I2C)
 *   Port 0 range1
 *   Port 1 range2
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="GS Arm")
//@Disabled
public class GS_arm extends LinearOpMode {

    //Values that converts encoder ticks to degrees
    static final double COUNTS_PER_MOTOR_REV = 28;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 100.0;     // This is < 1.0 if geared UP
    static final double COUNTS_PER_DEGREE = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (360);
    // names the Motors, Servos, Sensors, and Pins
    DcMotor l_arm = null;
    DcMotor u_arm = null;
    DcMotor s_arm = null;
    Servo hand = null;
    TouchSensor touch;
    DistanceSensor sensorRange1;
    DistanceSensor sensorRange2;
    DigitalChannel headin;
    DigitalChannel headout;

    @Override
    public void runOpMode() { //Main Code


        // Initialize and Reset the Encoders.
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        // Mapping the hardware component names to the name used in the code
        l_arm = hardwareMap.dcMotor.get("l_arm");
        u_arm = hardwareMap.dcMotor.get("u_arm");
        s_arm = hardwareMap.dcMotor.get("s_arm");
        hand = hardwareMap.servo.get("hand");
        touch = hardwareMap.touchSensor.get("touch");
        headin = hardwareMap.digitalChannel.get("headin");
        headout = hardwareMap.digitalChannel.get("headout");

        // Setting the mode on each hardware device
        // Motors
        l_arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        u_arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        s_arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        l_arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        u_arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        s_arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Digital Pins
        headin.setMode(DigitalChannel.Mode.INPUT);
        headout.setMode(DigitalChannel.Mode.OUTPUT);

        // Initializing the variables used to define the target location of each arm system
        int newl_armTarget;
        int newu_armTarget;
        int news_armTarget;

        // Defining the Motors again using DcMotorEx for more data and control over DcMotors
        DcMotorEx l_arm1 = hardwareMap.get(DcMotorEx.class, "l_arm");
        DcMotorEx u_arm1 = hardwareMap.get(DcMotorEx.class, "u_arm");
        DcMotorEx s_arm1 = hardwareMap.get(DcMotorEx.class, "s_arm");

        // Defining the REV 2M Distance Sensors
        sensorRange1 = hardwareMap.get(DistanceSensor.class, "range1");
        sensorRange2 = hardwareMap.get(DistanceSensor.class, "range2");

        // Setting Safety limits on Motors
        /* l_arm > 3.5 AMPS    u_arm > 7.5 AMPS    s_arm  > 5.5 */
        l_arm1.setCurrentAlert(3.5, CurrentUnit.AMPS);
        u_arm1.setCurrentAlert(7.5, CurrentUnit.AMPS);
        s_arm1.setCurrentAlert(5.5, CurrentUnit.AMPS);

        //Setting Servo to 0 position
        hand.setPosition(0.5);

        // Wait for App Init
        waitForStart();

        // Setting Motor speed/power to 60%
        double speed = 0.6;

        headout.setState(false); // Set Output Pin Low

        //While the code is running
        while (opModeIsActive()) {
            // Prints the initial Obstruction condition of the robot arm.
            if (sensorRange1.getDistance(DistanceUnit.CM) > 50 && sensorRange2.getDistance(DistanceUnit.CM) > 50) {
                telemetry.addData("Clear to RUN", 0);
            }
            if (sensorRange1.getDistance(DistanceUnit.CM) < 50) {
                telemetry.addData("Obstruction Detected on the Side", 1);
            }
            if (sensorRange2.getDistance(DistanceUnit.CM) < 50) {
                telemetry.addData("Obstruction Detected on the Front", 1);
            }
                telemetry.update();


            //Reset Arm Position
            newl_armTarget = 0;
            l_arm.setTargetPosition(newl_armTarget);
            l_arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            l_arm.setPower(Math.abs(speed));

            newu_armTarget = 0;
            u_arm.setTargetPosition(newu_armTarget);
            u_arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            u_arm.setPower(Math.abs(speed));

            s_arm.setPower(0);

            //when the touch sensor is pushed or Input pin high when no obstacles closer than 50cm
            if (touch.isPressed() == true && sensorRange1.getDistance(DistanceUnit.CM) > 50 && sensorRange2.getDistance(DistanceUnit.CM) > 50 || headin.getState()==false && sensorRange1.getDistance(DistanceUnit.CM) > 50 && sensorRange2.getDistance(DistanceUnit.CM) > 50 ) {


                //The lower arm holds its position during the first phase
                newl_armTarget = l_arm.getCurrentPosition();
                l_arm.setTargetPosition(newl_armTarget);
                l_arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                l_arm.setPower(Math.abs(speed));

                //First Phase the upper arm swings out 90 degrees
                newu_armTarget = u_arm.getCurrentPosition() + (int) (-90 * COUNTS_PER_DEGREE);
                u_arm.setTargetPosition(newu_armTarget);
                u_arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                u_arm.setPower(Math.abs(speed));

                // Saftey System for Phase one
                while (u_arm.isBusy() && opModeIsActive()) {
                    if (u_arm1.isOverCurrent() == true || sensorRange1.getDistance(DistanceUnit.CM) < 50 || sensorRange2.getDistance(DistanceUnit.CM) < 50 ) {
                        u_arm1.setMotorDisable();
                        headout.setState(true);
                        requestOpModeStop();
                    }
                }

                //The lower arm holds its position during the first phase
                newl_armTarget = l_arm.getCurrentPosition();
                l_arm.setTargetPosition(newl_armTarget);
                l_arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                l_arm.setPower(Math.abs(speed));

                //wait 1 sec
                sleep(1000);

                //Second Phase bends the lower arm from the elbow downwards 90 degree
                newl_armTarget = l_arm.getCurrentPosition() + (int) ((90*3/5) *COUNTS_PER_DEGREE);
                l_arm.setTargetPosition(newl_armTarget);
                l_arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                l_arm.setPower(Math.abs(speed));

                // Saftey System for Phase two
                while (l_arm.isBusy() && opModeIsActive()) {
                    if (l_arm1.isOverCurrent() == true || sensorRange1.getDistance(DistanceUnit.CM) < 50 || sensorRange2.getDistance(DistanceUnit.CM) < 50) {
                        l_arm1.setMotorDisable();
                        headout.setState(true);
                        requestOpModeStop();
                    }
                }

                //wait 1 sec
                sleep(1000);

                // hold this new position for lower arm
                newl_armTarget = l_arm.getCurrentPosition();
                l_arm.setTargetPosition(newl_armTarget);
                l_arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                l_arm.setPower(Math.abs(speed));

                // hold this new position for upper arm
                newu_armTarget = u_arm.getCurrentPosition();
                u_arm.setTargetPosition(newu_armTarget);
                u_arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                u_arm.setPower(Math.abs(speed));

                //Third Phase rotates the shoulder 180 degrees
                news_armTarget = s_arm.getCurrentPosition() + (int) (180 * COUNTS_PER_DEGREE);
                s_arm.setTargetPosition(news_armTarget);
                s_arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                s_arm.setPower(Math.abs(0.15));

                // Saftey System for Phase three
                while (s_arm.isBusy() && opModeIsActive()) {
                    if (s_arm1.isOverCurrent() == true  || sensorRange2.getDistance(DistanceUnit.CM) < 50) {
                        s_arm1.setMotorDisable();
                        headout.setState(true);
                        requestOpModeStop();
                    }
                }
                // pushes the upper arm an extra 30 degrees
                newu_armTarget = u_arm.getCurrentPosition() + (int) (30 * COUNTS_PER_DEGREE);
                u_arm.setTargetPosition(newu_armTarget);
                u_arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                u_arm.setPower(Math.abs(0.2));

                // Saftey System for the extra push
                while (u_arm.isBusy() && opModeIsActive()) {
                    if (u_arm1.isOverCurrent() == true || sensorRange2.getDistance(DistanceUnit.CM) < 50) {
                        u_arm1.setMotorDisable();
                        headout.setState(true);
                        requestOpModeStop();
                    }
                }

                //wait 1 sec
                sleep(1000);

                // makes GS hand sign
                hand.setPosition(1.0);

                // hold position for 5 sec
                sleep(5000);

                // resets hand
                hand.setPosition(0.5);

                //Undoes Third Phase
                news_armTarget = s_arm.getCurrentPosition() + (int) (-180 * COUNTS_PER_DEGREE);
                s_arm.setTargetPosition(news_armTarget);
                s_arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                s_arm.setPower(Math.abs(0.15));

                // Saftey System for Phase three
                while (s_arm.isBusy() && opModeIsActive()) {
                    if (s_arm1.isOverCurrent() == true  || sensorRange1.getDistance(DistanceUnit.CM) < 50) {
                        s_arm1.setMotorDisable();
                        headout.setState(true);
                        requestOpModeStop();
                    }
                }

                //wait 1 sec
                sleep(1000);

                //Undoes Phase two
                newu_armTarget = u_arm.getCurrentPosition();
                u_arm.setTargetPosition(newu_armTarget);
                u_arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                u_arm.setPower(Math.abs(speed));

                newl_armTarget = l_arm.getCurrentPosition() + (int) ((-90*3/5)* COUNTS_PER_DEGREE);
                l_arm.setTargetPosition(newl_armTarget);
                l_arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                l_arm.setPower(Math.abs(speed));

                // Saftey System for Phase two
                while (l_arm.isBusy() && opModeIsActive()) {
                    if (l_arm1.isOverCurrent() == true  || sensorRange1.getDistance(DistanceUnit.CM) < 50 || sensorRange2.getDistance(DistanceUnit.CM) < 50) {
                        l_arm1.setMotorDisable();
                        headout.setState(true);
                        requestOpModeStop();
                    }
                }

                //wait 1 sec
                sleep(1000);

                // Undoes Phase one
                newl_armTarget = l_arm.getCurrentPosition();
                l_arm.setTargetPosition(newl_armTarget);
                l_arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                l_arm.setPower(Math.abs(speed));

                newu_armTarget = u_arm.getCurrentPosition() + (int) (90 * COUNTS_PER_DEGREE);
                u_arm.setTargetPosition(newu_armTarget);
                u_arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                u_arm.setPower(Math.abs(0.3));

                // Saftey System for Phase one
                u_arm1.setCurrentAlert(8, CurrentUnit.AMPS);
                while (u_arm.isBusy() && opModeIsActive()) {
                    if (u_arm1.isOverCurrent() == true  || sensorRange1.getDistance(DistanceUnit.CM) < 50 || sensorRange2.getDistance(DistanceUnit.CM) < 50) {
                        u_arm1.setMotorDisable();
                        headout.setState(true);
                        requestOpModeStop();
                    }
                }

                sleep(1000);   // optional pause after each move////
            }

        }
    }
}

