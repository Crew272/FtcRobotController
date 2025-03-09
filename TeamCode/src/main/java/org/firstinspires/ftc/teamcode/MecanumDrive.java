package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp(name = "Mecanum Drive with Reassigned Grabber Controls", group = "TeleOp")
public class MecanumDrive extends LinearOpMode {

    private RobotHardware robot;
    private boolean upAndDownState = false; // Tracks toggle state for upAndDownServo
    private boolean previousBState = false; // Tracks the previous state of the B button
    private boolean claw2GrabState = false; // Tracks the toggle state for Claw2 Grab Servo
    private boolean previousAState = false; // Tracks the previous state of the A button

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotHardware(hardwareMap);

        telemetry.addLine("Ready for start");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // **Driver Controls (Gamepad A)**
            double drive = -gamepad1.left_stick_y; // Forward/Backward
            double strafe = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rotate = gamepad1.right_stick_x; // Rotation

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(rotate), 1);
            double frontLeftPower = (drive + strafe + rotate) / denominator;
            double frontRightPower = (drive - strafe - rotate) / denominator;
            double rearLeftPower = (drive - strafe + rotate) / denominator;
            double rearRightPower = (drive + strafe - rotate)/ denominator;


            robot.frontLeftDrive.setPower(frontLeftPower);
            robot.frontRightDrive.setPower(frontRightPower);
            robot.rearLeftDrive.setPower(rearLeftPower);
            robot.rearRightDrive.setPower(rearRightPower);

            // **Operator Controls (Gamepad B)**

            // Grabber 1
            robot.grabber1LiftMotor.setPower(-gamepad2.left_stick_y);

            if (gamepad2.left_bumper) {
                robot.arm1RotationServo.setPosition(0.0); // Rotate Arm1 Left
            } else if (gamepad2.right_bumper) {
                robot.arm1RotationServo.setPosition(1.0); // Rotate Arm1 Right
            }

            if (gamepad2.left_trigger > 0.5) {
                robot.claw1GrabServo.setPosition(0.0); // Open Claw1
            } else if (gamepad2.right_trigger > 0.5) {
                robot.claw1GrabServo.setPosition(1.0); // Close Claw1
            }

            // Grabber 2
            double slidePower = gamepad2.right_stick_y;
            robot.slideLeftServo.setPosition(0.5 + slidePower / 2); // Slide Left Servo
            robot.slideRightServo.setPosition(0.5 - slidePower / 2); // Slide Right Servo

            double claw2Rotation = gamepad2.right_stick_x;
            if (claw2Rotation != 0) {
                robot.claw2RotationServo.setPosition(0.5 + (claw2Rotation / 2)); // Adjust Claw2 Rotation
            }

            // Claw2 Grab Toggle using A Button
            if (gamepad2.a && !previousAState) {
                claw2GrabState = !claw2GrabState;
                robot.claw2GrabServo.setPosition(claw2GrabState ? 1.0 : 0.0);
            }
            previousAState = gamepad2.a;

            // UpAndDownServo Toggle using B Button
            if (gamepad2.b && !previousBState) {
                upAndDownState = !upAndDownState;
                robot.upAndDownServo.setPosition(upAndDownState ? 1.0 : 0.0);
            }
            previousBState = gamepad2.b;

            // Telemetry for Debugging
            telemetry.addData("Claw2 Grab State", claw2GrabState ? "Closed" : "Open");
            telemetry.addData("UpAndDownServo State", upAndDownState ? "Up" : "Down");
            telemetry.addData("Slide Power", slidePower);
            telemetry.addData("Claw2 Rotation", claw2Rotation);
            telemetry.update();
        }
    }
}
