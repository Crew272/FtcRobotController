package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Basic: Autonomous", group="Basic")
public class AutonomousBasic extends LinearOpMode {

    private RobotHardwareBasic robot = new RobotHardwareBasic();
    private ElapsedTime runtime = new ElapsedTime();

    // Movement parameters
    private static final double DRIVE_SPEED = 0.5;
    private static final double TURN_SPEED = 0.4;
    private static final double STRAFE_SPEED = 0.5;

    // Robot physical constants
    private static final double ROBOT_DIAMETER_INCHES = 18.0; // Adjust based on your robot

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting
        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // Example autonomous sequence
            driveForward(24.0);    // Drive forward 24 inches
            sleep(250);            // Pause for stability
            turnRight(90.0);       // Turn 90 degrees right
            sleep(250);
            driveForward(12.0);    // Drive forward 12 inches
            sleep(250);
            strafeRight(12.0);     // Strafe right 12 inches

            telemetry.addData("Path", "Complete");
            telemetry.update();
        }
    }

    public void driveForward(double inches) {
        int targetCounts = (int)(inches * robot.COUNTS_PER_INCH);

        // Reset encoders
        robot.resetEncoders();

        // Set target position
        robot.setTargetPosition(targetCounts);

        // Set to RUN_TO_POSITION mode
        robot.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Start moving
        robot.setPower(DRIVE_SPEED);

        // Wait for movement to complete
        while (opModeIsActive() && robot.isBusy()) {
            telemetry.addData("Moving", "Forward: %2.1f inches", inches);
            telemetry.addData("Target", targetCounts);
            telemetry.addData("Current Pos", robot.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion
        robot.setPower(0);
        robot.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Optional pause
        sleep(250);
    }

    public void driveBackward(double inches) {
        driveForward(-inches);
    }

    public void strafeRight(double inches) {
        int targetCounts = (int)(inches * robot.COUNTS_PER_INCH);

        robot.resetEncoders();

        // Set individual targets for strafing
        robot.frontLeftDrive.setTargetPosition(targetCounts);
        robot.frontRightDrive.setTargetPosition(-targetCounts);
        robot.rearLeftDrive.setTargetPosition(-targetCounts);
        robot.rearRightDrive.setTargetPosition(targetCounts);

        robot.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set power for strafing
        robot.setIndividualPowers(STRAFE_SPEED, -STRAFE_SPEED, -STRAFE_SPEED, STRAFE_SPEED);

        // Wait for movement to complete
        while (opModeIsActive() && robot.isBusy()) {
            telemetry.addData("Strafing", "Right: %2.1f inches", inches);
            telemetry.addData("Current Pos", robot.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion
        robot.setPower(0);
        robot.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(250);
    }

    public void strafeLeft(double inches) {
        strafeRight(-inches);
    }

    public void turnRight(double degrees) {
        // Calculate the number of encoder counts for the turn
        double inches = (degrees / 360.0) * Math.PI * ROBOT_DIAMETER_INCHES;
        int targetCounts = (int)(inches * robot.COUNTS_PER_INCH);

        robot.resetEncoders();

        // Set individual targets for turning
        robot.frontLeftDrive.setTargetPosition(targetCounts);
        robot.frontRightDrive.setTargetPosition(-targetCounts);
        robot.rearLeftDrive.setTargetPosition(targetCounts);
        robot.rearRightDrive.setTargetPosition(-targetCounts);

        robot.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set power for turning
        robot.setIndividualPowers(TURN_SPEED, -TURN_SPEED, TURN_SPEED, -TURN_SPEED);

        // Wait for movement to complete
        while (opModeIsActive() && robot.isBusy()) {
            telemetry.addData("Turning", "Right: %2.1f degrees", degrees);
            telemetry.addData("Target", targetCounts);
            telemetry.addData("Current Pos", robot.getCurrentPosition());
            logMotorPositions();
            telemetry.update();
        }

        // Stop all motion
        robot.setPower(0);
        robot.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(250);
    }

    public void turnLeft(double degrees) {
        turnRight(-degrees);
    }

    private void logMotorPositions() {
        telemetry.addData("FL Position", robot.frontLeftDrive.getCurrentPosition());
        telemetry.addData("FR Position", robot.frontRightDrive.getCurrentPosition());
        telemetry.addData("RL Position", robot.rearLeftDrive.getCurrentPosition());
        telemetry.addData("RR Position", robot.rearRightDrive.getCurrentPosition());
    }

    // Test sequence for tuning PID
    public void runPIDTuningTest() {
        if (opModeIsActive()) {
            // Forward and back test
            driveForward(24);  // Drive forward 24 inches
            sleep(1000);
            driveBackward(24); // Drive back 24 inches
            sleep(1000);

            // Rotation test
            turnRight(90);     // Turn 90 degrees right
            sleep(1000);
            turnLeft(90);      // Turn 90 degrees left
            sleep(1000);

            // Strafe test
            strafeRight(12);   // Strafe right 12 inches
            sleep(1000);
            strafeLeft(12);    // Strafe left 12 inches
            sleep(1000);
        }
    }
}
