package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Autonomous Drive", group = "Autonomous")
public class AutonomousDrive extends LinearOpMode {

    private RobotHardware robot;
    private final double TICKS_PER_INCH = 7.288; //23.094; // Adjusted for updated specs
    private double arm1RotationPosition = 0.9; // Start at .9
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize RobotHardware
        robot = new RobotHardware(hardwareMap);

        // Reset encoders and set to RUN_USING_ENCODER mode
        resetEncoders();
        setRunModeUsingEncoders();
        // Initialize the servo position before the loop starts
        robot.arm1RotationServo.setPosition(arm1RotationPosition);
        robot.claw1GrabServo.setPosition(1.0); //close claw on specimen
        telemetry.addLine("Ready for start");
        telemetry.update();

        // Wait for the game to start
        waitForStart();

        // Execute movements
        driveForward(36);    // Drive forward 1'
        //strafeRight(10);     // Strafe right 2' 1"
        //driveForward(39);    // Drive forward 3' 3"
        //strafeRight(10);     // Strafe right 10"
        //driveBackward(12);   // Backward 4' 5"
        //driveForward(53);    // Forward 4' 5"
        //strafeRight(9);      // Strafe right 9"
        //driveBackward(24);   // Backward 4' 5"
        //driveForward(53);    // Forward 4' 5"
        //strafeRight(7);      // Strafe right 7"
        //driveBackward(53);   // Backward 4' 5"
        //strafeLeft(36);
        robot.claw1GrabServo.setPosition(1.0); // Keep claw closed at end
        telemetry.addLine("Autonomous complete!");
        telemetry.update();
    }

    private void resetEncoders() {
        robot.frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rearLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rearRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void setRunModeUsingEncoders() {
        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void driveForward(double inches) {
        moveRobot(inches, inches, inches, inches);
    }

    private void driveBackward(double inches) {
        moveRobot(-inches, -inches, -inches, -inches);
    }

    private void strafeRight(double inches) {
        moveRobot(inches, -inches, -inches, inches);
    }

    private void strafeLeft(double inches) {
        moveRobot(-inches, inches, inches, -inches);
    }

    private void moveRobot(double flInches, double frInches, double rlInches, double rrInches) {
        int flTarget = robot.frontLeftDrive.getCurrentPosition() + (int) (flInches * TICKS_PER_INCH);
        int frTarget = robot.frontRightDrive.getCurrentPosition() + (int) (frInches * TICKS_PER_INCH);
        int rlTarget = robot.rearLeftDrive.getCurrentPosition() + (int) (rlInches * TICKS_PER_INCH);
        int rrTarget = robot.rearRightDrive.getCurrentPosition() + (int) (rrInches * TICKS_PER_INCH);

        robot.frontLeftDrive.setTargetPosition(flTarget);
        robot.frontRightDrive.setTargetPosition(frTarget);
        robot.rearLeftDrive.setTargetPosition(rlTarget);
        robot.rearRightDrive.setTargetPosition(rrTarget);

        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rearLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rearRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Adjust power levels for smoother strafe movement
        double flPower = 0.5;
        double frPower = 0.5;
        double rlPower = 0.5;
        double rrPower = 0.5;

        // If strafing, reduce diagonal wheel power slightly
        if (flInches == -frInches && rlInches == -rrInches) {
            flPower = 0.6;
            frPower = 0.4;
            rlPower = 0.4;
            rrPower = 0.6;
        }

        robot.frontLeftDrive.setPower(flPower);
        robot.frontRightDrive.setPower(frPower);
        robot.rearLeftDrive.setPower(rlPower);
        robot.rearRightDrive.setPower(rrPower);

//        robot.frontLeftDrive.setPower(0.5);
//        robot.frontRightDrive.setPower(0.5);
//        robot.rearLeftDrive.setPower(0.5);
//        robot.rearRightDrive.setPower(0.5);

        while (opModeIsActive() && robot.frontLeftDrive.isBusy() && robot.frontRightDrive.isBusy()
                && robot.rearLeftDrive.isBusy() && robot.rearRightDrive.isBusy()) {
            telemetry.addData("FL Target", flTarget);
            telemetry.addData("FL Position", robot.frontLeftDrive.getCurrentPosition());
            telemetry.addData("FR Target", frTarget);
            telemetry.addData("FR Position", robot.frontRightDrive.getCurrentPosition());
            telemetry.addData("RL Target", rlTarget);
            telemetry.addData("RL Position", robot.rearLeftDrive.getCurrentPosition());
            telemetry.addData("RR Target", rrTarget);
            telemetry.addData("RR Position", robot.rearRightDrive.getCurrentPosition());
            telemetry.update();
        }

        stopMotors();
        setRunModeUsingEncoders();
    }

    private void stopMotors() {
        robot.frontLeftDrive.setPower(0);
        robot.frontRightDrive.setPower(0);
        robot.rearLeftDrive.setPower(0);
        robot.rearRightDrive.setPower(0);
    }
}
