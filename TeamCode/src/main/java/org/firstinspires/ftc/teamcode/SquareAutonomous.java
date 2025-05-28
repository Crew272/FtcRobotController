package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Square Autonomous", group="Robot")
public class SquareAutonomous extends LinearOpMode {
    // Declare hardware
    private RobotHardwareAu robot = new RobotHardwareAu();

    // Constants
    private static final double WHEEL_DIAMETER_INCHES = 3.78; // 96mm goBILDA mecanum
    private static final double TICKS_PER_REV = 537.6; // NeveRest 20
    private static final double TICKS_PER_INCH = TICKS_PER_REV / (WHEEL_DIAMETER_INCHES * Math.PI);
    private static final int DISTANCE_TICKS = (int) (12 * TICKS_PER_INCH); // 2 feet
    private static final double DRIVE_POWER = 0.5;

    @Override
    public void runOpMode() {
        // Initialize hardware
        robot.init(hardwareMap);

        // Wait for start
        waitForStart();

        // Drive square: forward, strafe left, backward, strafe right
        driveForward(DISTANCE_TICKS);
        sleep(500); // Brief pause
        strafeLeft(DISTANCE_TICKS);
        sleep(500);
        driveBackward(DISTANCE_TICKS);
        sleep(500);
        strafeRight(DISTANCE_TICKS);
    }

    private void driveForward(int ticks) {
        setTargetPosition(ticks, ticks, ticks, ticks);
        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        setPower(DRIVE_POWER, DRIVE_POWER, DRIVE_POWER, DRIVE_POWER);
        waitForMotors();
        stopMotors();
    }

    private void driveBackward(int ticks) {
        setTargetPosition(-ticks, -ticks, -ticks, -ticks);
        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        setPower(-DRIVE_POWER, -DRIVE_POWER, -DRIVE_POWER, -DRIVE_POWER);
        waitForMotors();
        stopMotors();
    }

    private void strafeLeft(int ticks) {
        setTargetPosition(-ticks, ticks, ticks, -ticks);
        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        setPower(-DRIVE_POWER, DRIVE_POWER, DRIVE_POWER, -DRIVE_POWER);
        waitForMotors();
        stopMotors();
    }

    private void strafeRight(int ticks) {
        setTargetPosition(ticks, -ticks, -ticks, ticks);
        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        setPower(DRIVE_POWER, -DRIVE_POWER, -DRIVE_POWER, DRIVE_POWER);
        waitForMotors();
        stopMotors();
    }

    private void setTargetPosition(int flTicks, int blTicks, int frTicks, int brTicks) {
        robot.frontLeft.setTargetPosition(robot.frontLeft.getCurrentPosition() + flTicks);
        robot.backLeft.setTargetPosition(robot.backLeft.getCurrentPosition() + blTicks);
        robot.frontRight.setTargetPosition(robot.frontRight.getCurrentPosition() + frTicks);
        robot.backRight.setTargetPosition(robot.backRight.getCurrentPosition() + brTicks);
    }

    private void setMotorMode(DcMotor.RunMode mode) {
        robot.frontLeft.setMode(mode);
        robot.backLeft.setMode(mode);
        robot.frontRight.setMode(mode);
        robot.backRight.setMode(mode);
    }

    private void setPower(double flPower, double blPower, double frPower, double brPower) {
        robot.frontLeft.setPower(flPower);
        robot.backLeft.setPower(blPower);
        robot.frontRight.setPower(frPower);
        robot.backRight.setPower(brPower);
    }

    private void waitForMotors() {
        while (opModeIsActive() && (
                robot.frontLeft.isBusy() ||
                        robot.backLeft.isBusy() ||
                        robot.frontRight.isBusy() ||
                        robot.backRight.isBusy())) {
            // Wait until motors reach target
        }
    }

    private void stopMotors() {
        setPower(0, 0, 0, 0);
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
