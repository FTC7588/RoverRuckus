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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  rearLeftDrive:        "rearLeftDrive"
 * Motor channel:  rearRightDrive:       "rearRightDrive"
 * Motor channel:  frontLeftDrive:       "frontLeftDrive"
 * Motor channel:  frontRightDrive:      "frontRightDrive"
 * Servo channel:  Servo to move left claw: "leftClaw"
 * Servo channel:  Servo to move right claw: "rightClaw"
 */
public class HardwareRoverBot
{
    /* Public OpMode members. */
    public DcMotor  rearLeftDrive   = null;
    public DcMotor  rearRightDrive  = null;
    public DcMotor  frontLeftDrive  = null;
    public DcMotor  frontRightDrive  = null;
    public DcMotor  climber = null;
    public DcMotor  cubeArm = null;
    public Servo mineralPusher = null;
    public Servo mineralIntake = null;
    public Servo hatch = null;
    public ColorSensor colorSensor = null;

    public static final double MID_SERVO = 0.5 ;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;

    /* Constructor */
    public HardwareRoverBot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        //set an initialize sensors
        colorSensor = hwMap.get(ColorSensor.class, "ColorSensor");

        // Define and Initialize Servos
        mineralPusher = hwMap.get(Servo.class, "mineralPusher");
        mineralIntake = hwMap.get(Servo.class, "mineralIntake");
        hatch = hwMap.get(Servo.class, "hatch");

        mineralPusher.setPosition(0);
        mineralIntake.setPosition(MID_SERVO);
        hatch.setPosition(MID_SERVO);

        // Define and Initialize Motors
        rearLeftDrive   = hwMap.get(DcMotor.class, "rearLeftDrive");
        rearRightDrive  = hwMap.get(DcMotor.class, "rearRightDrive");
        frontLeftDrive  = hwMap.get(DcMotor.class, "frontLeftDrive");
        frontRightDrive = hwMap.get(DcMotor.class, "frontRightDrive");
        climber =hwMap.get(DcMotor.class, "climber");
        cubeArm =hwMap.get(DcMotor.class, "cubeArm");

        rearLeftDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rearRightDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        climber.setDirection(DcMotor.Direction.FORWARD);
        cubeArm.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        rearLeftDrive.setPower(0);
        rearRightDrive.setPower(0);
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        climber.setPower(0);
        cubeArm.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        rearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        climber.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        cubeArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rearLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        climber.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        cubeArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
 }

