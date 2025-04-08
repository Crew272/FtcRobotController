package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Autonomous Square Test", group = "Linear OpMode")
public class AutonomousBasic extends LinearOpMode {

    private RobotHardwareBasic robot = new RobotHardwareBasic();

    @Override
    public void runOpMode() {
        // Initialize hardware (includes odometry)
        robot.init(hardwareMap);

        // Reset odometry before the match starts
        robot.resetPosAndIMU();

        telemetry.addData("Status", "Initialized & Odometry Reset");
        telemetry.addData("Starting X (mm)", robot.getCurrentX());
        telemetry.addData("Starting Y (mm)", robot.getCurrentY());
        telemetry.addData("Starting Heading (°)", robot.getCurrentHeading());
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // Move forward 24 inches
            moveForward(24, 0.5);

            // Move left 12 inches
            moveLeft(12, 0.5);

            // Move backward 24 inches
            moveForward(-24, 0.5);

            // Move right 12 inches
            moveLeft(-12, 0.5);
        }
    }

    // Move forward/backward a specific distance in inches
    private void moveForward(double targetInches, double power) {
        double targetY = targetInches * 25.4; // Convert inches to mm
        double Kp = 0.005;
        double Ki = 0.0;
        double Kd = 0.0;

        double integral = 0;
        double previousError = 0;
        final double POSITION_TOLERANCE_MM = 5.0; // Stop within 5mm (~0.2 inches)
        final double TIMEOUT_SECONDS = 5.0; // Stop after 5 seconds
        final double INTEGRAL_BOUND = 100.0; // **Limit integral to ±100**
        double startTime = getRuntime();
        double startY = robot.getCurrentY();

        while (opModeIsActive()) {
            // Update odometry
            robot.update();
            double currentY = robot.getCurrentY();

            // Calculate error
            double error = targetY + startY - currentY;

            // Stop if within tolerance or if timeout is reached
            if (Math.abs(error) < POSITION_TOLERANCE_MM || (getRuntime() - startTime > TIMEOUT_SECONDS)) {
                stopMotors();
                telemetry.addData("Status", "Arrived at target or timed out.");
                telemetry.update();
                break;
            }

            // PID calculations
            integral += error;
            integral = Math.max(-INTEGRAL_BOUND, Math.min(INTEGRAL_BOUND, integral)); // **Clamp integral**


            double derivative = error - previousError;
            double drivePower = (Kp * error) + (Ki * integral) + (Kd * derivative);

            // Cap power to prevent overshoot
            drivePower = Math.max(-power, Math.min(power, drivePower));

            // Apply power to all drive motors (forward movement)
            robot.frontLeft.setPower(drivePower);
            robot.frontRight.setPower(drivePower);
            robot.rearLeft.setPower(drivePower);
            robot.rearRight.setPower(drivePower);

            previousError = error;

            // Debugging telemetry
            telemetry.addData("Target Y (mm)", targetY + startY);
            telemetry.addData("Current Y (mm)", currentY);
            telemetry.addData("Error Y (mm)", error);
            telemetry.addData("Drive Power", drivePower);
            telemetry.addData("Time Elapsed", getRuntime() - startTime);
            telemetry.update();
        }
        // **Keep telemetry up for 30 seconds after stopping**
        double stopTime = getRuntime();
        while (opModeIsActive() && getRuntime() - stopTime < 30) {
            telemetry.addData("Final Status", "Stopped - Holding telemetry for review.");
            telemetry.addData("Final Target Y (mm)", targetY + startY);
            telemetry.addData("Final Position Y (mm)", robot.getCurrentY());
            telemetry.addData("Final Error Y (mm)", targetY + startY - robot.getCurrentY());
            telemetry.addData("Final Time Elapsed", getRuntime() - startTime);
            telemetry.update();
        }
    }

    // Move left/right a specific distance in inches
    private void moveLeft(double targetInches, double power) {
        double targetX = targetInches * 25.4; // Convert inches to mm
        double Kp = 0.005;
        double Ki = 0.0;
        double Kd = 0.0;

        double integral = 0;
        double previousError = 0;
        final double POSITION_TOLERANCE_MM = 5.0; // Stop within 5mm (~0.2 inches)
        final double TIMEOUT_SECONDS = 5.0; // Stop after 5 seconds
        final double INTEGRAL_BOUND = 100.0; // **Limit integral to ±100**
        double startTime = getRuntime();
        double startX = robot.getCurrentX();

        while (opModeIsActive()) {
            // Update odometry
            robot.update();
            double currentX = robot.getCurrentX();

            // Calculate error
            double error = targetX + startX - currentX;

            // Stop if within tolerance or if timeout is reached
            if (Math.abs(error) < POSITION_TOLERANCE_MM || (getRuntime() - startTime > TIMEOUT_SECONDS)) {
                stopMotors();
                telemetry.addData("Status", "Arrived at target or timed out.");
                telemetry.update();
                break;
            }

            // PID calculations
            integral += error;
            integral = Math.max(-INTEGRAL_BOUND, Math.min(INTEGRAL_BOUND, integral)); // **Clamp integral**


            double derivative = error - previousError;
            double drivePower = (Kp * error) + (Ki * integral) + (Kd * derivative);

            // Cap power to prevent overshoot
            drivePower = Math.max(-power, Math.min(power, drivePower));

            // Apply power to all drive motors (strafing movement)
            robot.frontLeft.setPower(-drivePower);
            robot.frontRight.setPower(drivePower);
            robot.rearLeft.setPower(drivePower);
            robot.rearRight.setPower(-drivePower);

            previousError = error;

            // Debugging telemetry
            telemetry.addData("Target X (mm)", targetX + startX);
            telemetry.addData("Current X (mm)", currentX);
            telemetry.addData("Error X (mm)", error);
            telemetry.addData("Drive Power", drivePower);
            telemetry.addData("Time Elapsed", getRuntime() - startTime);
            telemetry.update();
        }
        // **Keep telemetry up for 30 seconds after stopping**
        double stopTime = getRuntime();
        while (opModeIsActive() && getRuntime() - stopTime < 30) {
            telemetry.addData("Final Status", "Stopped - Holding telemetry for review.");
            telemetry.addData("Final Target X (mm)", targetX + startX);
            telemetry.addData("Final Position X (mm)", robot.getCurrentX());
            telemetry.addData("Final Error X (mm)", targetX + startX - robot.getCurrentX());
            telemetry.addData("Final Time Elapsed", getRuntime() - startTime);
            telemetry.update();
        }
    }
    private void stopMotors() {
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.rearLeft.setPower(0);
        robot.rearRight.setPower(0);
        telemetry.addData("Status", "Stopped.");
        telemetry.update();
    }
}

