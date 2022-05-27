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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * This file will be needed to be run beofre the acutally running the GS arm code for the Interactive Lego Girl Scout Robot Arm
 *
 * The code REQUIRES that you DO have encoders on the motors that are 28 tick
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
 *   Port 1 range1
 *   Port 2 range2
 *   Port 3 range3
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Precheck")
//@Disabled
public class Precheck extends LinearOpMode {

    //Values that converts encoder ticks to degrees
    static final double COUNTS_PER_MOTOR_REV = 28;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 100.0;     // This is < 1.0 if geared UP
    static final double COUNTS_PER_DEGREE = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (360);
    // names the Motors, Servos, and Sensors
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
    public void runOpMode() {


        // Initialize and Reset the Encoders.
        telemetry.addData("Status", "Resetting Encoders");    //
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
        l_arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        u_arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        s_arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        l_arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        u_arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        s_arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
        sensorRange1 = hardwareMap.get(DistanceSensor.class, "range1");
        sensorRange2 = hardwareMap.get(DistanceSensor.class, "range2");

        // Setting Safety limits on Motors
        /* l_arm > 3.5 AMPS    u_arm > 6.5 AMPS    s_arm  > 4.5 */
        l_arm1.setCurrentAlert(3.5, CurrentUnit.AMPS);
        u_arm1.setCurrentAlert(6.5, CurrentUnit.AMPS);
        s_arm1.setCurrentAlert(4.5, CurrentUnit.AMPS);

        //Setting Servo to 0 position
        hand.setPosition(0.5);

        // Wait for App Init
        waitForStart();

        // Setting Motor speed/power to 20%
        double speed = 0.2;

        //While the code is running
        while (opModeIsActive()) {
            // Prints the inital outputs of current to the app screen
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

            news_armTarget = 0;
            s_arm.setTargetPosition(news_armTarget);
            s_arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            s_arm.setPower(Math.abs(0.2));

            //when the touch sensor is pushed
            if (touch.isPressed() == true && sensorRange1.getDistance(DistanceUnit.CM) > 50) {


                //The lower arm direction test
                newl_armTarget = l_arm.getCurrentPosition() + (int) ((-30*3/5) * COUNTS_PER_DEGREE);
                l_arm.setTargetPosition(newl_armTarget);
                l_arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                l_arm.setPower(Math.abs(speed));

                while (u_arm.isBusy() && opModeIsActive()) {
                    if (u_arm1.isOverCurrent() == true || sensorRange1.getDistance(DistanceUnit.CM) < 50) {
                        u_arm1.setMotorDisable();
                        requestOpModeStop();
                    }
                }

                sleep(1000);

                while (opModeIsActive()) {

                    if (touch.isPressed() == true && sensorRange1.getDistance(DistanceUnit.CM) > 50) {
                        //The upper arm direction test
                        newu_armTarget = u_arm.getCurrentPosition() + (int) (-30 * COUNTS_PER_DEGREE);
                        u_arm.setTargetPosition(newu_armTarget);
                        u_arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        u_arm.setPower(Math.abs(speed));

                        // Saftey System for Phase one
                        while (u_arm.isBusy() && opModeIsActive()) {
                            if (u_arm1.isOverCurrent() == true || sensorRange1.getDistance(DistanceUnit.CM) < 50) {
                                u_arm1.setMotorDisable();
                                requestOpModeStop();
                            }
                        }
                        sleep(1000);
                        while (opModeIsActive()) {
                            if (touch.isPressed() == true && sensorRange1.getDistance(DistanceUnit.CM) > 50) {
                                //The shoulder
                                news_armTarget = s_arm.getCurrentPosition() + (int) (30 * COUNTS_PER_DEGREE);
                                s_arm.setTargetPosition(news_armTarget);
                                s_arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                s_arm.setPower(Math.abs(0.2));

                                // Saftey System for Phase three
                                while (s_arm.isBusy() && opModeIsActive()) {
                                    if (s_arm1.isOverCurrent() == true || sensorRange1.getDistance(DistanceUnit.CM) < 50) {
                                        s_arm1.setMotorDisable();
                                        requestOpModeStop();
                                    }
                                }
                                sleep(5000);
                                requestOpModeStop();
                            }
                        }
                    }
                }
            }
        }
    }
}




