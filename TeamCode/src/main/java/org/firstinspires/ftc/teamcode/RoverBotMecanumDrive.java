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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import static android.os.SystemClock.sleep;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Rover Bot Mecanum Drive", group="Teleop")
//@Disabled
public class RoverBotMecanumDrive extends OpMode{

    double x;
    double y;
    double r;
    double cubePower;

    /* Declare OpMode members. */
    HardwareRoverBot robot       = new HardwareRoverBot(); // use the class created to define a Pushbot's hardware
                                                         // could also use HardwarePushbotMatrix class.

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Oi Mate");    //
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        if (gamepad1.left_trigger > .5) {
            robot.rearLeftDrive.setPower(-1.0);
            robot.rearRightDrive.setPower(1.0);
            robot.frontLeftDrive.setPower(1.0);
            robot.frontRightDrive.setPower(-1.0);
        } else if (gamepad1.right_trigger > .5) {
            robot.rearLeftDrive.setPower(1.0);
            robot.rearRightDrive.setPower(-1.0);
            robot.frontLeftDrive.setPower(-1.0);
            robot.frontRightDrive.setPower(1.0);
        } else {
            //Mecanum test here
            x = -gamepad1.left_stick_y;
            y = 0;
            r = -gamepad1.right_stick_x;

            //robot.frontLeftDrive.setPower(Range.scale(x - y + r, -2.0, 2.0, -1.0, 1.0));
            //robot.frontRightDrive.setPower(Range.scale(x + y - r, -2.0, 2.0, -1.0, 1.0));
            //robot.rearLeftDrive.setPower(Range.scale(x + y + r, -2.0, 2.0, -1.0, 1.0));
            //robot.rearRightDrive.setPower(Range.scale(x - y - r, -2.0, 2.0, -1.0, 1.0));

            robot.frontLeftDrive.setPower(Range.clip(x - y + r, -1.0, 1.0));
            robot.frontRightDrive.setPower(Range.clip(x + y - r, -1.0, 1.0));
            robot.rearLeftDrive.setPower(Range.clip(x + y + r, -1.0, 1.0));
            robot.rearRightDrive.setPower(Range.clip(x - y - r, -1.0, 1.0));
        }

        //control the climber
        if (gamepad2.y) {
            robot.climber.setPower(1);
        } else if (gamepad2.a) {
            robot.climber.setPower(-1);
        } else {
            robot.climber.setPower(0);
        }

        //control the team marker
        if (gamepad2.back) {
            robot.markerArm.setPosition(0);
        } else if (gamepad2.start) {
            robot.markerArm.setPosition(robot.MID_SERVO);
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}