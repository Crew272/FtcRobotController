package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Autonomous Forward Test", group = "Linear OpMode")
public class AutonomousBasic extends LinearOpMode {

    private RobotHardwareBasic robot = new RobotHardwareBasic();

    @Override
    public void runOpMode() {
        // Initialize hardware (includes odometry)
        robot.init(hardwareMap);

        // Reset odometry before the match starts
        robot.odo.resetPosAndIMU();

        telemetry.addData("Status", "Initialized & Odometry Reset");
        telemetry.addData("Starting X (mm)", robot.getCurrentPose().getX(DistanceUnit.MM));
        telemetry.addData("Starting Y (mm)", robot.getCurrentPose().getY(DistanceUnit.MM));
        telemetry.addData("Starting Heading (°)", robot.getCurrentPose().getHeading(AngleUnit.DEGREES));
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // Move forward 24 inches
            moveForward(24, 0.5);
        }
    }

    // Move forward a specific distance in inches
    private void moveForward(double targetInches, double power) {
        double targetX = targetInches * 25.4; // Convert inches to mm
        double Kp = 0.005;
        double Ki = 0.0;
        double Kd = 0.0;

        double integral = 0;
        double previousError = 0;
        final double POSITION_TOLERANCE_MM = 5.0; // Stop within 5mm (~0.2 inches)
        final double TIMEOUT_SECONDS = 5.0; // Stop after 5 seconds

        double startTime = getRuntime();

        while (opModeIsActive()) {
            // Update odometry
            robot.odo.update();
            Pose2D currentPose = robot.getCurrentPose();
            double currentX = currentPose.getX(DistanceUnit.MM);

            // Calculate error
            double error = targetX - currentX;

            // Stop if within tolerance or if timeout is reached
            if (Math.abs(error) < POSITION_TOLERANCE_MM || (getRuntime() - startTime > TIMEOUT_SECONDS)) {
                stopMotors();
                telemetry.addData("Status", "Arrived at target or timed out.");
                telemetry.update();
                break;
            }

            // PID calculations
            integral += error;
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
            telemetry.addData("Target X (mm)", targetX);
            telemetry.addData("Current X (mm)", currentX);
            telemetry.addData("Error X (mm)", error);
            telemetry.addData("Drive Power", drivePower);
            telemetry.addData("Time Elapsed", getRuntime() - startTime);
            telemetry.update();
        }
    }

    // Stop all motors
    private void stopMotors() {
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.rearLeft.setPower(0);
        robot.rearRight.setPower(0);
    }
}
