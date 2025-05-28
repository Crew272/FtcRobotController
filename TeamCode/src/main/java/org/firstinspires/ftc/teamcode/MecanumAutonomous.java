package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="Mecanum Autonomous", group="Autonomous")
public class MecanumAutonomous extends LinearOpMode {

    // Declare motors
    private DcMotor leftFront;
    private DcMotor leftRear;
    private DcMotor rightFront;
    private DcMotor rightRear;

    // Constants for encoder calculations
    private static final double TICKS_PER_REV = 537.6; // NeverRest 20
    private static final double WHEEL_CIRCUMFERENCE = 4.0 * Math.PI; // 4" diameter
    private static final double TICKS_PER_INCH = TICKS_PER_REV / WHEEL_CIRCUMFERENCE; // ~42.8 ticks/inch

    @Override
    public void runOpMode() {
        // Initialize motors
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");

        // Set motor directions (adjust based on your wiring)
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRear.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set all motors to use encoders
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for start
        waitForStart();

        // Execute movements
        driveForward(24.0);   // Forward 24 inches
        strafeRight(12.0);    // Strafe right 12 inches
        driveBackward(24.0);  // Backward 24 inches
        strafeLeft(12.0);     // Strafe left 12 inches
    }

    // Drive forward a given distance in inches
    private void driveForward(double distance) {
        int ticks = (int) (distance * TICKS_PER_INCH);

        // Reset encoders
        resetEncoders();

        // Set target positions
        leftFront.setTargetPosition(ticks);
        leftRear.setTargetPosition(ticks);
        rightFront.setTargetPosition(ticks);
        rightRear.setTargetPosition(ticks);

        // Set mode to RUN_TO_POSITION
        setRunToPosition();

        // Set power
        setAllPowers(0.5);

        // Wait until motors reach position
        while (opModeIsActive() && (leftFront.isBusy() || leftRear.isBusy() ||
                rightFront.isBusy() || rightRear.isBusy())) {
            telemetry.addData("Driving Forward", "Target: %d", ticks);
            telemetry.update();
        }

        // Stop motors
        stopMotors();
    }

    // Drive backward a given distance in inches
    private void driveBackward(double distance) {
        int ticks = (int) (distance * TICKS_PER_INCH);

        // Reset encoders
        resetEncoders();

        // Set target positions (negative for backward)
        leftFront.setTargetPosition(-ticks);
        leftRear.setTargetPosition(-ticks);
        rightFront.setTargetPosition(-ticks);
        rightRear.setTargetPosition(-ticks);

        // Set mode to RUN_TO_POSITION
        setRunToPosition();

        // Set power
        setAllPowers(0.5);

        // Wait until motors reach position
        while (opModeIsActive() && (leftFront.isBusy() || leftRear.isBusy() ||
                rightFront.isBusy() || rightRear.isBusy())) {
            telemetry.addData("Driving Backward", "Target: %d", -ticks);
            telemetry.update();
        }

        // Stop motors
        stopMotors();
    }

    // Strafe right a given distance in inches
    private void strafeRight(double distance) {
        int ticks = (int) (distance * TICKS_PER_INCH);

        // Reset encoders
        resetEncoders();

        // Set target positions for strafing right
        leftFront.setTargetPosition(ticks);
        leftRear.setTargetPosition(-ticks);
        rightFront.setTargetPosition(-ticks);
        rightRear.setTargetPosition(ticks);

        // Set mode to RUN_TO_POSITION
        setRunToPosition();

        // Set power
        setAllPowers(0.5);

        // Wait until motors reach position
        while (opModeIsActive() && (leftFront.isBusy() || leftRear.isBusy() ||
                rightFront.isBusy() || rightRear.isBusy())) {
            telemetry.addData("Strafing Right", "Target: %d", ticks);
            telemetry.update();
        }

        // Stop motors
        stopMotors();
    }

    // Strafe left a given distance in inches
    private void strafeLeft(double distance) {
        int ticks = (int) (distance * TICKS_PER_INCH);

        // Reset encoders
        resetEncoders();

        // Set target positions for strafing left
        leftFront.setTargetPosition(-ticks);
        leftRear.setTargetPosition(ticks);
        rightFront.setTargetPosition(ticks);
        rightRear.setTargetPosition(-ticks);

        // Set mode to RUN_TO_POSITION
        setRunToPosition();

        // Set power
        setAllPowers(0.5);

        // Wait until motors reach position
        while (opModeIsActive() && (leftFront.isBusy() || leftRear.isBusy() ||
                rightFront.isBusy() || rightRear.isBusy())) {
            telemetry.addData("Strafing Left", "Target: %d", ticks);
            telemetry.update();
        }

        // Stop motors
        stopMotors();
    }

    // Helper method to reset encoders
    private void resetEncoders() {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Helper method to set RUN_TO_POSITION mode
    private void setRunToPosition() {
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // Helper method to set power for all motors
    private void setAllPowers(double power) {
        leftFront.setPower(power);
        leftRear.setPower(power);
        rightFront.setPower(power);
        rightRear.setPower(power);
    }

    // Helper method to stop all motors
    private void stopMotors() {
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
    }
}