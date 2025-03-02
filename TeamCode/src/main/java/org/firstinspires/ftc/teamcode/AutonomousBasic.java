package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Autonomous Basic with Mecanum", group = "Autonomous")
public class AutonomousBasic extends LinearOpMode {

    private RobotHardwareBasic robot;
    private static final double DRIVE_SPEED = 0.5;
    private static final double STRAFE_SPEED = 0.5;
    private static final double TURN_SPEED = 0.4;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotHardwareBasic(hardwareMap);

        telemetry.addLine("Robot Ready!");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // Example autonomous sequence
            driveForward(12.0);    // Drive forward 12 inches
            sleep(250);            // Brief pause between movements
            turnRight(90);         // Turn 90 degrees right
            sleep(250);
            strafeRight(24.0);     // Strafe right 24 inches
            sleep(250);
            driveBackward(12.0);   // Drive backward 12 inches
        }
    }

    private void driveForward(double inches) {
        moveRobot(inches, Direction.FORWARD);
    }

    private void driveBackward(double inches) {
        moveRobot(inches, Direction.BACKWARD);
    }

    private void strafeLeft(double inches) {
        moveRobot(inches, Direction.STRAFE_LEFT);
    }

    private void strafeRight(double inches) {
        moveRobot(inches, Direction.STRAFE_RIGHT);
    }

    public void turnRight(double degrees) {
        rotate(degrees);
    }

    public void turnLeft(double degrees) {
        rotate(-degrees);
    }

    private enum Direction {
        FORWARD,
        BACKWARD,
        STRAFE_LEFT,
        STRAFE_RIGHT
    }

    private void moveRobot(double inches, Direction direction) {
        int counts = (int)(inches * robot.COUNTS_PER_INCH);

        robot.resetEncoders();

        switch (direction) {
            case FORWARD:
                setTargetForAll(counts);
                break;
            case BACKWARD:
                setTargetForAll(-counts);
                break;
            case STRAFE_LEFT:
                robot.frontLeftDrive.setTargetPosition(-counts);
                robot.frontRightDrive.setTargetPosition(counts);
                robot.rearLeftDrive.setTargetPosition(counts);
                robot.rearRightDrive.setTargetPosition(-counts);
                break;
            case STRAFE_RIGHT:
                robot.frontLeftDrive.setTargetPosition(counts);
                robot.frontRightDrive.setTargetPosition(-counts);
                robot.rearLeftDrive.setTargetPosition(-counts);
                robot.rearRightDrive.setTargetPosition(counts);
                break;
        }

        robot.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        switch (direction) {
            case FORWARD:
            case BACKWARD:
                robot.setPower(DRIVE_SPEED);
                break;
            case STRAFE_LEFT:
            case STRAFE_RIGHT:
                robot.setPower(STRAFE_SPEED);
                break;
        }

        while (opModeIsActive() && robot.isBusy()) {
            telemetry.addData("Moving", direction.toString());
            telemetry.addData("Target", counts);
            telemetry.addData("Current Pos", robot.getCurrentPosition());
            telemetry.update();
        }

        robot.setPower(0);
        robot.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void rotate(double degrees) {
        int counts = (int)(degrees * robot.COUNTS_PER_DEGREE);

        robot.resetEncoders();

        // Set target positions for rotation
        robot.frontLeftDrive.setTargetPosition(counts);
        robot.frontRightDrive.setTargetPosition(-counts);
        robot.rearLeftDrive.setTargetPosition(counts);
        robot.rearRightDrive.setTargetPosition(-counts);

        robot.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set power for rotation
        robot.setRotationPowers(TURN_SPEED);

        while (opModeIsActive() && robot.isBusy()) {
            telemetry.addData("Rotating", "%.2f degrees", degrees);
            telemetry.addData("Target", counts);
            telemetry.addData("Current Pos", robot.getCurrentPosition());
            telemetry.update();
        }

        robot.setPower(0);
        robot.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sleep(250); // Small delay to ensure rotation is complete
    }

    private void setTargetForAll(int counts) {
        robot.frontLeftDrive.setTargetPosition(counts);
        robot.frontRightDrive.setTargetPosition(counts);
        robot.rearLeftDrive.setTargetPosition(counts);
        robot.rearRightDrive.setTargetPosition(counts);
    }
}

