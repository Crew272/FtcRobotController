package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

@Autonomous(name="Heart Path Autonomous", group="Robot")
public class HeartPathAutonomous extends LinearOpMode {
    private RobotHardware robot = new RobotHardware();
    private static final double WHEEL_DIAMETER_INCHES = 3.78; // 96mm
    private static final double TICKS_PER_REV = 537.6; // NeveRest 20
    private static final double TICKS_PER_INCH = TICKS_PER_REV / (WHEEL_DIAMETER_INCHES * Math.PI);
    private static final double POWER = 0.3; // Low power for control
    private static final double PATH_SCALE = 0.67; // Scale 3-foot heart to ~2 feet

    // Bezier control points for heart shape (in inches, scaled to ~2 feet)
    private static final double[][] LEFT_HALF = {
            {0, 0},                    // P0: Bottom point
            {0, 18 * PATH_SCALE},      // P1: Control point for left curve
            {-18 * PATH_SCALE, 36 * PATH_SCALE}, // P2: Control point for top left
            {-9 * PATH_SCALE, 36 * PATH_SCALE}   // P3: Top center
    };
    private static final double[][] RIGHT_HALF = {
            {-9 * PATH_SCALE, 36 * PATH_SCALE},  // P0: Top center
            {0, 36 * PATH_SCALE},                // P1: Control point for top right
            {18 * PATH_SCALE, 18 * PATH_SCALE},  // P2: Control point for right curve
            {0, 0}                               // P3: Bottom point
    };

    @Override
    public void runOpMode() {
        // Initialize hardware and telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        try {
            robot.init(hardwareMap);
            sleep(2000); // IMU calibration
        } catch (Exception e) {
            RobotLog.ee("HeartPathAutonomous", "Hardware init failed: %s", e.getMessage());
            return;
        }

        telemetry.addData("Status", "Ready to start");
        telemetry.update();
        waitForStart();
        ElapsedTime timer = new ElapsedTime();

        // Break heart into discrete segments (10 per half)
        int segments = 10;
        for (int i = 0; i <= segments && opModeIsActive(); i++) {
            double u = i / (double) segments;
            double[] pos = bezierPoint(LEFT_HALF, u);
            moveToPosition(pos[0], pos[1], POWER);
            telemetry.addData("Segment", "Left %d, X: %.2f, Y: %.2f", i, pos[0], pos[1]);
            telemetry.update();
            sleep(500); // Pause for stability
        }
        for (int i = 0; i <= segments && opModeIsActive(); i++) {
            double u = i / (double) segments;
            double[] pos = bezierPoint(RIGHT_HALF, u);
            moveToPosition(pos[0], pos[1], POWER);
            telemetry.addData("Segment", "Right %d, X: %.2f, Y: %.2f", i, pos[0], pos[1]);
            telemetry.update();
            sleep(500);
        }

        // Stop motors
        setPower(0, 0, 0, 0);
        telemetry.addData("Status", "Complete");
        telemetry.update();
    }

    // Move to position using RUN_TO_POSITION
    private void moveToPosition(double targetX, double targetY, double power) {
        // Current position
        double currentX = (robot.frontLeft.getCurrentPosition() + robot.frontRight.getCurrentPosition() +
                robot.backLeft.getCurrentPosition() + robot.backRight.getCurrentPosition()) / (4.0 * TICKS_PER_INCH);
        double currentY = (robot.frontLeft.getCurrentPosition() - robot.frontRight.getCurrentPosition() +
                robot.backLeft.getCurrentPosition() - robot.backRight.getCurrentPosition()) / (4.0 * TICKS_PER_INCH);

        // Calculate deltas
        double deltaX = targetX - currentX;
        double deltaY = targetY - currentY;

        // Convert to encoder ticks
        int ticksX = (int) (deltaX * TICKS_PER_INCH);
        int ticksY = (int) (deltaY * TICKS_PER_INCH);

        // Set target positions
        robot.frontLeft.setTargetPosition(robot.frontLeft.getCurrentPosition() + ticksX + ticksY);
        robot.backLeft.setTargetPosition(robot.backLeft.getCurrentPosition() + ticksX + ticksY);
        robot.frontRight.setTargetPosition(robot.frontRight.getCurrentPosition() + ticksX - ticksY);
        robot.backRight.setTargetPosition(robot.backRight.getCurrentPosition() + ticksX - ticksY);

        // Set mode and power
        robot.frontLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.backLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.frontRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.backRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        setPower(power, power, power, power);

        // Wait until movement completes
        while (opModeIsActive() && (robot.frontLeft.isBusy() || robot.backLeft.isBusy() ||
                robot.frontRight.isBusy() || robot.backRight.isBusy())) {
            telemetry.addData("Moving", "X: %.2f, Y: %.2f", currentX, currentY);
            telemetry.update();
        }

        // Stop and reset mode
        setPower(0, 0, 0, 0);
        robot.frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.backRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    // Cubic Bezier curve evaluation
    private double[] bezierPoint(double[][] controlPoints, double t) {
        double x = Math.pow(1 - t, 3) * controlPoints[0][0] +
                3 * Math.pow(1 - t, 2) * t * controlPoints[1][0] +
                3 * (1 - t) * t * t * controlPoints[2][0] +
                t * t * t * controlPoints[3][0];
        double y = Math.pow(1 - t,