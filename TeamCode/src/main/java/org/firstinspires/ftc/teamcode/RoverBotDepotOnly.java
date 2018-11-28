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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Rover Bot Depot Only", group="Auto")
//@Disabled
public class RoverBotDepotOnly extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareRoverBot robot = new HardwareRoverBot();   // Use a Pushbot's hardware

    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double SPOOL_DIAMETER_INCHES = 1.625;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double ELEVATOR_COUNTS_PER_INCH = COUNTS_PER_MOTOR_REV / (SPOOL_DIAMETER_INCHES * 3.1415);

    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    static final double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.1;     // Larger is more responsive, but also less stable
    static final double ELEVATOR_SPEED = 0.5;

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "AXPTQvj/////AAABmfe8VdJO1Ut0vWLwZbZkXIpoHvWlLYIv4ANU52SZaGL+f7Gz4a58VdLEGTBceJChEUwcgSqkOftzl47rP0bDKgjriyfQaXBfCufFEW+kCUpaHMRJzepjuUSETNJCfTSo+Ho2ZR4KUmQVpIDuINh0kZe3o/Kx3qMSgcXpKTwyfbe0qivpYCMa+LN6IpQJnmlxCB3gxtiYzD57jA4rVIs/PiDfCOBtDyD6P82Tf5QAqzdK4gIx7xMcMNjOCsJqlXduZB+iFtAGU8sQvhVMEEI5pT00nnR8XMtO24EQGRxatnDtbdcZjhIQAXSn1hNvnWo6U79Ave4qRBmOM0NNGwx42cN50jpG0eV2xzBhN6wplyxF";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private String goldPos;

    ElapsedTime timeOut = new ElapsedTime();

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        telemetry.addData("Status ", "Setting Up Tensor Flow");
        telemetry.update();

        setupTensor();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status ", "Resetting Encoders");
        telemetry.update();

        robot.frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.climber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.climber.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("IMU"," Calibrating");
        telemetry.update();

        while (!isStopRequested() && !robot.imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Status ", "Waiting For Start");
        telemetry.addData("IMU",robot.imu.getCalibrationStatus());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        initTensor();

        goldPos = sample();

        timeOut.reset();

        while (goldPos == "FAILED" && opModeIsActive() && timeOut.seconds() <= 5) {
            goldPos = sample();
        }

        elevatorMove(ELEVATOR_SPEED, 14, 10);

        //strafe("LEFT", 1.0, 750);

        gyroTurn(TURN_SPEED, -25);

        encoderDrive(.1, 1, 1, 1);

        gyroTurn(TURN_SPEED, 0);

        elevatorMove(ELEVATOR_SPEED, -13.5, 10);

        if (goldPos == "LEFT") {
            telemetry.addData("TensorFlow","LEFT");
            telemetry.update();

            sleep(2000);

            /**gyroTurn(TURN_SPEED, 45);

            sleep(1000);

            gyroHold(TURN_SPEED, 45, .2);

            sleep(1000);

            encoderDrive(DRIVE_SPEED,27 , 27, 5);

            sleep(1000);

            gyroTurn(TURN_SPEED, -45);

            sleep(1000);

            encoderDrive(DRIVE_SPEED,25 , 25, 5);

            sleep(1000);*/
        } else if (goldPos == "CENTER") {
            telemetry.addData("TensorFlow","CENTER");
            telemetry.update();

            sleep(2000);

            /**gyroHold(TURN_SPEED, 0, .2);

            sleep(1000);

            encoderDrive(DRIVE_SPEED,24 , 24, 5);

            encoderDrive(DRIVE_SPEED,19 , 19, 5);

            sleep(1000);*/
        } else if (goldPos == "RIGHT") {
            telemetry.addData("TensorFlow","RIGHT");
            telemetry.update();

            sleep(2000);

            /**gyroTurn(TURN_SPEED, -45);

            sleep(1000);

            gyroHold(TURN_SPEED, -45, .2);

            sleep(1000);

            encoderDrive(DRIVE_SPEED,27 , 27, 5);

            sleep(1000);

            gyroTurn(TURN_SPEED, 45);

            sleep(1000);

            encoderDrive(DRIVE_SPEED,25 , 25, 5);

            sleep(1000);*/
        } else {
            telemetry.addData("TensorFLow"," Failed");
            telemetry.update();
        }

        //encoderDrive(DRIVE_SPEED, 36, 36, 10);

        //encoderDrive(DRIVE_SPEED, 12, 12, 10);

        //robot.markerArm.setPosition(.15);

        //sleep(1000);

        //robot.markerArm.setPosition(robot.MID_SERVO);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    public void elevatorMove(double speed, double inches, double timeoutS) {
        ElapsedTime runtime = new ElapsedTime();

        int newTarget = 0;
        boolean running = false;

        speed = Math.abs(speed);

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newTarget = robot.climber.getCurrentPosition() + (int) (inches * ELEVATOR_COUNTS_PER_INCH);

            // reset the timeout time and start motion.
            runtime.reset();

            if (inches >= robot.climber.getCurrentPosition()) {
                robot.climber.setPower(speed);
            } else {
                robot.climber.setPower(-speed);
            }

            running = true;

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() && (runtime.seconds() < timeoutS) && running) {

                if (robot.climber.getCurrentPosition() >= newTarget - 20 && robot.climber.getCurrentPosition() <= newTarget + 20) {
                    robot.climber.setPower(0);

                    running = false;
                }

                // Display it for the driver.
                telemetry.addData("Elevator Path", "Running to %7d", newTarget);
                telemetry.addData("Elevator Path", "Running at %7d", robot.climber.getCurrentPosition());
                telemetry.update();
            }

            robot.climber.setPower(0);
        }
    }

    public void strafe(String Direction, double power, int miliSecs) {

        if (Direction == "LEFT") {
            robot.rearLeftDrive.setPower(-power);
            robot.rearRightDrive.setPower(power);
            robot.frontLeftDrive.setPower(power);
            robot.frontRightDrive.setPower(-power);
        } else if (Direction == "RIGHT") {
            robot.rearLeftDrive.setPower(power);
            robot.rearRightDrive.setPower(-power);
            robot.frontLeftDrive.setPower(-power);
            robot.frontRightDrive.setPower(power);
        } else {
            telemetry.addData("Auto Strafe ", "Invalid Distance");
            telemetry.update();

            sleep(2500);

            return;
        }

        sleep(miliSecs);

        robot.rearLeftDrive.setPower(0);
        robot.rearRightDrive.setPower(0);
        robot.frontLeftDrive.setPower(0);
        robot.frontRightDrive.setPower(0);
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {

        ElapsedTime runtime = new ElapsedTime();
        int newLeftTarget = 0;
        int newRightTarget = 0;

        boolean leftRunning = false;
        boolean rightRunning = false;

        speed = Math.abs(speed);

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.frontLeftDrive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.frontRightDrive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);

            // reset the timeout time and start motion.
            runtime.reset();

            if (leftInches >= robot.frontLeftDrive.getCurrentPosition()) {
                robot.frontLeftDrive.setPower(speed);
                robot.rearLeftDrive.setPower(speed);
            } else {
                robot.frontLeftDrive.setPower(-speed);
                robot.rearLeftDrive.setPower(-speed);
            }

            if (rightInches >= robot.frontRightDrive.getCurrentPosition()) {
                robot.frontRightDrive.setPower(speed);
                robot.rearRightDrive.setPower(speed);
            } else {
                robot.frontRightDrive.setPower(-speed);
                robot.rearRightDrive.setPower(-speed);
            }

            leftRunning = true;
            rightRunning = true;

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() && (runtime.seconds() < timeoutS) &&
                    (leftRunning || rightRunning)) {

                if (robot.frontLeftDrive.getCurrentPosition() >= newLeftTarget) {
                    robot.frontLeftDrive.setPower(0);
                    robot.rearLeftDrive.setPower(0);

                    leftRunning = false;
                }

                sleep(50);

                if (robot.frontRightDrive.getCurrentPosition() >= newRightTarget) {
                    robot.rearRightDrive.setPower(0);
                    robot.frontRightDrive.setPower(0);

                    rightRunning = false;
                }

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d : %7d",
                        robot.frontLeftDrive.getCurrentPosition(),
                        robot.frontRightDrive.getCurrentPosition());
                telemetry.update();
            }

            robot.rearLeftDrive.setPower(0);
            robot.rearRightDrive.setPower(0);
            robot.frontLeftDrive.setPower(0);
            robot.frontRightDrive.setPower(0);
        }
    }

    /**
     * Method to spin on central axis to point in a new direction.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the heading (angle)
     * 2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle Absolute Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn(double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    /**
     * Method to obtain & hold a heading for a finite amount of time
     * Move will stop once the requested time has elapsed
     *
     * @param speed    Desired speed of turn.
     * @param angle    Absolute Angle (in Degrees) relative to last gyro reset.
     *                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                 If a relative angle is required, add/subtract from current heading.
     * @param holdTime Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold(double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        robot.rearLeftDrive.setPower(0);
        robot.rearRightDrive.setPower(0);
        robot.frontLeftDrive.setPower(0);
        robot.frontRightDrive.setPower(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed  Desired speed of turn.
     * @param angle  Absolute Angle (in Degrees) relative to last gyro reset.
     *               0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *               If a relative angle is required, add/subtract from current heading.
     * @param PCoeff Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double error;
        double steer;
        boolean onTarget = false;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        } else {
            steer = getSteer(error, PCoeff);
            rightSpeed = speed * steer;
            leftSpeed = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.rearLeftDrive.setPower(-leftSpeed);
        robot.rearRightDrive.setPower(-rightSpeed);
        robot.frontLeftDrive.setPower(-leftSpeed);
        robot.frontRightDrive.setPower(-rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     *
     * @param targetAngle Desired angle (relative to global reference established at last Gyro Reset).
     * @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     * +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - angles.firstAngle;
        while (robotError > 180 && opModeIsActive()) robotError -= 360;
        while (robotError <= -180 && opModeIsActive()) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     *
     * @param error  Error angle in robot relative degrees
     * @param PCoeff Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    public String sample() {
        String pos = "FAILED";

        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                if (updatedRecognitions.size() == 3) {
                    int goldMineralX = -1;
                    int silverMineral1X = -1;
                    int silverMineral2X = -1;
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            goldMineralX = (int) recognition.getLeft();
                        } else if (silverMineral1X == -1) {
                            silverMineral1X = (int) recognition.getLeft();
                        } else {
                            silverMineral2X = (int) recognition.getLeft();
                        }
                    }
                    if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                        if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                            telemetry.addData("Gold Mineral Position", "Left");
                            pos = "LEFT";
                        } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                            telemetry.addData("Gold Mineral Position", "Right");
                            pos = "RIGHT";
                        } else {
                            telemetry.addData("Gold Mineral Position", "Center");
                            pos = "CENTER";
                        }
                    }
                }
                telemetry.update();
            }
        }

        return pos;
    }

    public void setupTensor() {
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
    }

    public void initTensor() {
        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
}
