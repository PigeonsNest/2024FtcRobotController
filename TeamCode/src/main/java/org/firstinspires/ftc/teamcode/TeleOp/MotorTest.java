/* Copyright (c) 2021 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
//import org.firstinspires.ftc.teamcode.HardwareMechanum;


/**
 Checking motor configs
**/

@TeleOp(name="MotorTest", group="Linear Opmode")
public class MotorTest extends LinearOpMode {
        //HardwareMechanum robot = new HardwareMechanum();

        // Declare OpMode members for each of the 4 motors.
        private ElapsedTime runtime = new ElapsedTime();
        private DcMotor leftFrontDrive = null; //motor 3
        private DcMotor leftBackDrive = null; //motor 2
        private DcMotor rightBackDrive = null; //motor 1
        private DcMotor rightFrontDrive = null; //motor 0
        private DcMotor shoulderLeft = null;
        private DcMotor shoulderRight = null;
        private DcMotor forearm = null;
        private Servo servo = null;


        //Wrist stuff
        final double WRIST_INCREMENT = 0.001;
        final double WRIST_MAX = 1.0;
        final double WRIST_MIN = 0.25;
        double servoPosition = 0.0;

        //Finger stuff
        //bro i didn't know that's what he was saying to me as i was
        //going to my room bc i was cold :skull:
        //i hate to do this to him but i do have to comment this out
        //servo work better if it dont work like this
        // final double FINGER_INCREMENT = 0.002;
        //  final double CLAW_MAX = 1.0;
        final double CLAW_MIN = 0.8;
        // double leftFingerPosition = 0.0;
        // double rightFingerPosition = 0.0;

        double armPosition;
        double MIN_POSITION = 0;
        double MAX_POSITION = 1;
        double armOffset = 0;
        double ARM_SPEED = 0.02;

        @Override
        public void runOpMode() {

            //robot.init(hardwareMap);

            // Initialize the hardware variables. Note that the strings used here must correspond
            // to the names assigned during the robot configuration step on the DS or RC devices.
            leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive"); //motor 3
            leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive"); //motor 2
            rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive"); //motor 1
            rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive"); //motor 0
            shoulderLeft = hardwareMap.get(DcMotor.class, "shoulder_left"); //motor 0 expansion hub
            shoulderRight = hardwareMap.get(DcMotor.class, "shoulder_right"); //motor 1 expansion hub
            forearm = hardwareMap.get(DcMotor.class, "forearm"); //motor 2 expansion hub
            servo = hardwareMap.get(Servo.class, "servo"); //servo 1 on control hub

            leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
            rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
            rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

            //lift init stuff
            shoulderLeft.setDirection(DcMotor.Direction.FORWARD);
            shoulderLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            shoulderRight.setDirection(DcMotor.Direction.REVERSE);
            shoulderRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            forearm.setDirection(DcMotor.Direction.FORWARD);
            forearm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            servo = hardwareMap.get(Servo.class, "servo");


            // Wait for the game to start (driver presses PLAY)
            telemetry.addData("Status", "Initialized");
            telemetry.update();

            waitForStart();
            runtime.reset();

            // HEY,
            // To test more motors, change "rightBackDrive" to the name of the motor you want to test
            rightBackDrive.setPower(0.5);
            sleep(5000);
            rightBackDrive.setPower(0);
//
//            // Motor 00
//            if(gamepad1.a) {
//                leftFrontDrive.setPower(0.5);
//            }
//            // Motor 01
//            if(gamepad1.b){
//                leftBackDrive.setPower(0.5);
//            }
//            // Motor 02
//            if(gamepad1.x) {
//                rightFrontDrive.setPower(0.5);
//
//            }
//            // Motor 03
//            if(gamepad1.y){
//                rightBackDrive.setPower(0.5);
//            }
//            if (gamepad1.left_bumper) {
//                leftFrontDrive.setPower(0);
//                leftBackDrive.setPower(0);
//                rightFrontDrive.setPower(0);
//                rightBackDrive.setPower(0);
//            }
        }


}
