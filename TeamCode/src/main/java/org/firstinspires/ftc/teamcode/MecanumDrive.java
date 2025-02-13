package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Mecanum Drive with Reassigned Grabber Controls", group = "TeleOp")
public class MecanumDrive extends LinearOpMode {

    private RobotHardware robot;
    private double arm1RotationPosition = 0.8; // Start at .8
    private double upAndDownPosition = 0.4; // Start at .4
    private double slidePosition = 0.4;  // Start at .4 for slide, goes out to .57

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotHardware(hardwareMap);
        // Initialize the servo position before the loop starts
        robot.arm1RotationServo.setPosition(arm1RotationPosition);
        robot.upAndDownServo.setPosition(upAndDownPosition);
        robot.slideRightServo.setPosition(slidePosition);
        robot.claw1GrabServo.setPosition(1.0); //close claw on specimen
        telemetry.addLine("Ready for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double speedModifier = 0.5; // Reduce speed (1.0 = full speed, 0.3 = slow speed)
            double drive = -gamepad1.left_stick_y * speedModifier;
            double strafe = gamepad1.left_stick_x * speedModifier;
            double rotate = gamepad1.right_stick_x * speedModifier;

            double frontLeftPower = drive + strafe + rotate;
            double frontRightPower = drive - strafe - rotate;
            double rearLeftPower = drive - strafe + rotate;
            double rearRightPower = drive + strafe - rotate;

            double maxPower = Math.max(1.0, Math.max(
                    Math.abs(frontLeftPower),
                    Math.max(Math.abs(frontRightPower),
                            Math.max(Math.abs(rearLeftPower),
                                    Math.abs(rearRightPower)))));

            robot.frontLeftDrive.setPower(frontLeftPower / maxPower);
            robot.frontRightDrive.setPower(frontRightPower / maxPower);
            robot.rearLeftDrive.setPower(rearLeftPower / maxPower);
            robot.rearRightDrive.setPower(rearRightPower / maxPower);

            // **Operator Controls (Gamepad B)**

            // Grabber 1
            //robot.grabber1LiftMotor.setPower(-gamepad2.left_stick_y);
            double liftPower = gamepad2.left_stick_y; // Invert for correct movement
            // Reset encoder when the limit switch is pressed
            if (robot.isLiftAtBottom()) {
                robot.grabber1LiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.grabber1LiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            // Prevent downward movement if limit switch is engaged
            if (robot.isLiftAtBottom() && liftPower > 0) {
                robot.grabber1LiftMotor.setPower(0);
            }
            // Prevent going too high
            else if (robot.grabber1LiftMotor.getCurrentPosition() <= RobotHardware.LIFT_MAX_HEIGHT_TICKS && liftPower < 0) {
                robot.grabber1LiftMotor.setPower(0);
            }
            // Allow movement within limits
            else {
                robot.grabber1LiftMotor.setPower(liftPower);
            }

            // Read joystick X-axis
            double armMove = gamepad2.left_stick_x;

            // Deadzone to prevent jitter
            if (Math.abs(armMove) > 0.05) {
                // Quadratic scaling for smooth acceleration
                double movement = Math.signum(armMove) * Math.pow(Math.abs(armMove), 2) * 0.01; // Adjust for speed

                // Update servo position gradually based on joystick direction
                arm1RotationPosition += movement;
            }

            // Clamp position to servo limits (0.0 to 0.9)
            arm1RotationPosition = Math.max(0.0, Math.min(0.9, arm1RotationPosition));

            // Apply new position to servo
            robot.arm1RotationServo.setPosition(arm1RotationPosition);



            if (gamepad2.left_trigger > 0.5) {
                robot.claw1GrabServo.setPosition(0.0); // Open Claw1
            } else if (gamepad2.right_trigger > 0.5) {
                robot.claw1GrabServo.setPosition(1.0); // Close Claw1
            }

            // Grabber 2

            // Read joystick X-axis
            double slidePower = -gamepad2.right_stick_y;
            // Deadzone to prevent jitter
            if (Math.abs(slidePower) > 0.05) {
                // Quadratic scaling for smooth acceleration
                double movement = Math.signum(slidePower) * Math.pow(Math.abs(slidePower), 2) * 0.01; // Adjust for speed

                // Update servo position gradually based on joystick direction
                slidePosition += movement;
            }

            // Clamp position to servo limits (0.0 to 1.0)
            slidePosition = Math.max(0.4, Math.min(0.57, slidePosition));

            // Apply new position to servo
            robot.slideRightServo.setPosition(slidePosition);

            // Read joystick X-axis
            double upAndDownArmMove = gamepad2.right_stick_x;

            // Deadzone to prevent jitter
            if (Math.abs(upAndDownArmMove) > 0.05) {
                // Quadratic scaling for smooth acceleration
                double movement = Math.signum(upAndDownArmMove) * Math.pow(Math.abs(upAndDownArmMove), 2) * 0.01; // Adjust for speed

                // Update servo position gradually based on joystick direction
                upAndDownPosition += movement;
            }

            // Clamp position to servo limits (0.0 to 1.0)
            upAndDownPosition = Math.max(0.0, Math.min(0.4, upAndDownPosition));

            // Apply new position to servo
            robot.upAndDownServo.setPosition(upAndDownPosition);

            // **NEW: Intake Control (Continuous Rotation Servo)**
            if (gamepad2.left_bumper) {
                robot.intake1.setPower(-1.0);  // Rotate Counterclockwise
            } else if (gamepad2.right_bumper) {
                robot.intake1.setPower(1.0);   // Rotate Clockwise
            } else {
                robot.intake1.setPower(0.0);   // Stop rotation
            }

            // Telemetry for Debugging
            telemetry.addData("Slide Position", slidePosition);
            telemetry.addData("Arm Rotation Servo Position", arm1RotationPosition);
            telemetry.addData("Up and Down", upAndDownPosition);
            telemetry.addData("Lift Position", robot.grabber1LiftMotor.getCurrentPosition());
            telemetry.addData("Limit Switch", robot.isLiftAtBottom() ? "Pressed" : "Not Pressed");
            telemetry.update();
        }
    }
}
