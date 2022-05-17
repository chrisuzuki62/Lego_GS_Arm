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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * This file will drive the entire Interactive Lego Girl Scout Robot Arm
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

@Autonomous(name="Test2")
//@Disabled
public class Test1 extends LinearOpMode {

    //Values that converts encoder ticks to degrees
    static final double     COUNTS_PER_MOTOR_REV    = 28 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 60.0 ;     // This is < 1.0 if geared UP
    static final double     COUNTS_PER_DEGREE         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (360);
    // names the Motors, Servos, and Sensors
    DcMotor l_arm = null;
    DcMotor u_arm = null;
    DcMotor s_arm = null;
    Servo hand = null;
    TouchSensor touch;

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

        // Setting the mode on each hardware device
        l_arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        u_arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        s_arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        l_arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        u_arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        s_arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initializing the variables used to define the target location of each arm system
        int newl_armTarget;
        int newu_armTarget;
        int news_armTarget;

        // Defining the Motors again using DcMotorEx for more data and control over DcMotors
        DcMotorEx l_arm1 = hardwareMap.get(DcMotorEx.class, "l_arm");
        DcMotorEx u_arm1 = hardwareMap.get(DcMotorEx.class, "u_arm");
        DcMotorEx s_arm1 = hardwareMap.get(DcMotorEx.class, "s_arm");

        //Setting Servo to 0 position
        hand.setPosition(0.5);

        // Wait for App Init
        waitForStart();

        // Setting Motor speed/power to 60%
        double speed = 0.6;

        //While the code is running
        while(opModeIsActive()) {
            // Prints the inital outputs of current to the app screen
            telemetry.addData("l_arm Current", l_arm1.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("u_arm Current", u_arm1.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("s_arm Current", s_arm1.getCurrent(CurrentUnit.AMPS));
            telemetry.update();

            //when the touch sensor is pushed
            if (touch.isPressed() == true) {

                //First Phase the upper arm swings out 90 degrees
                newu_armTarget = u_arm.getCurrentPosition() + (int) (-90 * COUNTS_PER_DEGREE);
                u_arm.setTargetPosition(newu_armTarget);
                u_arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                u_arm.setPower(Math.abs(speed));

                telemetry.addData("l_arm Current", l_arm1.getCurrent(CurrentUnit.AMPS));
                telemetry.addData("u_arm Current", u_arm1.getCurrent(CurrentUnit.AMPS));
                telemetry.addData("s_arm Current", s_arm1.getCurrent(CurrentUnit.AMPS));
                telemetry.update();

                //The lower arm holds its position during the first phase
                newl_armTarget = l_arm.getCurrentPosition();
                l_arm.setTargetPosition(newl_armTarget);
                l_arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                l_arm.setPower(Math.abs(speed));

                //Test code for motor resistance safety system
                if (u_arm1.getCurrent(CurrentUnit.AMPS) > 6.5 || l_arm1.getCurrent(CurrentUnit.AMPS) > 3.5 || s_arm1.getCurrent(CurrentUnit.AMPS) > 4) {
                    requestOpModeStop();
                }
                //wait 1 sec
                sleep(1000);

                //Second Phase bends the lower arm from the elbow downwards
                newl_armTarget = l_arm.getCurrentPosition() + (int) (90 * COUNTS_PER_DEGREE);
                l_arm.setTargetPosition(newl_armTarget);
                l_arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                l_arm.setPower(Math.abs(speed));

                sleep(1000);   // optional pause after each move

                // hold this new position
                newl_armTarget = l_arm.getCurrentPosition();
                l_arm.setTargetPosition(newl_armTarget);
                l_arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                l_arm.setPower(Math.abs(speed));

                // raise the upper arm abit since it sags on rotation of the shoulder
                newu_armTarget = u_arm.getCurrentPosition() + (int) (30 * COUNTS_PER_DEGREE);
                u_arm.setTargetPosition(newu_armTarget);
                u_arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                u_arm.setPower(Math.abs(speed));

                //Third Phase rotates the shoulder 180 degrees
                news_armTarget = s_arm.getCurrentPosition() + (int) (180 * COUNTS_PER_DEGREE);
                s_arm.setTargetPosition(news_armTarget);
                s_arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                s_arm.setPower(Math.abs(0.2));

                sleep(1000);   // optional pause after each move

                // makes GS hand sign
                hand.setPosition(1.0);

                sleep(5000);

                // resets hand
                hand.setPosition(0.5);

                //Undoes Third Phase
                news_armTarget = s_arm.getCurrentPosition() + (int) (-180 * COUNTS_PER_DEGREE);
                s_arm.setTargetPosition(news_armTarget);
                s_arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                s_arm.setPower(Math.abs(speed));

                newu_armTarget = u_arm.getCurrentPosition() + (int) (-30 * COUNTS_PER_DEGREE);
                u_arm.setTargetPosition(newu_armTarget);
                u_arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                u_arm.setPower(Math.abs(speed));

                sleep(1000);   // optional pause after each move

                //Undoes Phase two
                newl_armTarget = l_arm.getCurrentPosition() + (int) (-90 * COUNTS_PER_DEGREE);
                l_arm.setTargetPosition(newl_armTarget);
                l_arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                l_arm.setPower(Math.abs(speed));

                sleep(1000);   // optional pause after each move

                // Undoes Phase one
                newl_armTarget = l_arm.getCurrentPosition();
                l_arm.setTargetPosition(newl_armTarget);
                l_arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                l_arm.setPower(Math.abs(speed));

                newu_armTarget = u_arm.getCurrentPosition() + (int) (89 * COUNTS_PER_DEGREE);
                u_arm.setTargetPosition(newu_armTarget);
                u_arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                u_arm.setPower(Math.abs(0.2));

                sleep(1000);   // optional pause after each move////
            }
            else {

            }

        }
    }
}
