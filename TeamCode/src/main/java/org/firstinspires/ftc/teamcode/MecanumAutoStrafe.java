package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Strafe Right 1 Foot", group = "Mecanum")
public class MecanumAutoStrafe extends LinearOpMode {

    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor rearLeftDrive = null;
    private DcMotor rearRightDrive = null;

    private ElapsedTime runtime = new ElapsedTime();

    // Constants for encoder calculations
    static final double COUNTS_PER_MOTOR_REV = 537.6; // Example for NeveRest Orbital 20
    static final double WHEEL_DIAMETER_INCHES = 4.0;  // Wheel diameter in inches
    static final double COUNTS_PER_INCH = COUNTS_PER_MOTOR_REV / (Math.PI * WHEEL_DIAMETER_INCHES);
    static final double STRAFE_SPEED = 0.5;

    @Override
    public void runOpMode() {
        // Initialize hardware
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        rearLeftDrive = hardwareMap.get(DcMotor.class, "rearLeftDrive");
        rearRightDrive = hardwareMap.get(DcMotor.class, "rearRightDrive");

        // Set motor directions
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        rearLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        rearRightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Reset encoders and set to RUN_USING_ENCODER mode
        resetEncoders();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the start command
        waitForStart();

        if (opModeIsActive()) {
            // Strafe right 1 foot (12 inches)
            encoderStrafe(STRAFE_SPEED, 12, 5.0); // Speed, distance, timeout in seconds
            telemetry.addData("Path", "Complete");
            telemetry.update();
        }
    }

    /**
     * Method to strafe a specific distance using encoders.
     * @param speed Motor speed (0.0 to 1.0)
     * @param distanceInInches Distance to strafe in inches
     * @param timeoutS Timeout in seconds
     */
    private void encoderStrafe(double speed, double distanceInInches, double timeoutS) {
        int targetPosition = (int) (distanceInInches * COUNTS_PER_INCH);

        // Calculate target positions for strafing
        int frontLeftTarget = targetPosition;   // Move forward
        int frontRightTarget = -targetPosition; // Move backward
        int rearLeftTarget = -targetPosition;   // Move backward
        int rearRightTarget = targetPosition;   // Move forward

        // Set target positions
        frontLeftDrive.setTargetPosition(frontLeftTarget);
        frontRightDrive.setTargetPosition(frontRightTarget);
        rearLeftDrive.setTargetPosition(rearLeftTarget);
        rearRightDrive.setTargetPosition(rearRightTarget);

        // Set to RUN_TO_POSITION mode
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Start motion
        runtime.reset();
        frontLeftDrive.setPower(speed);
        frontRightDrive.setPower(speed);
        rearLeftDrive.setPower(speed);
        rearRightDrive.setPower(speed);

        // Wait until target is reached or timeout
        while (opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                (frontLeftDrive.isBusy() && frontRightDrive.isBusy() &&
                        rearLeftDrive.isBusy() && rearRightDrive.isBusy())) {

            // Telemetry for debugging
            telemetry.addData("Target Position", targetPosition);
            telemetry.addData("Front Left", frontLeftDrive.getCurrentPosition());
            telemetry.addData("Front Right", frontRightDrive.getCurrentPosition());
            telemetry.addData("Rear Left", rearLeftDrive.getCurrentPosition());
            telemetry.addData("Rear Right", rearRightDrive.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion
        stopAllMotors();

        // Reset motors to RUN_USING_ENCODER mode
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Resets all motor encoders.
     */
    private void resetEncoders() {
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Stops all motors.
     */
    private void stopAllMotors() {
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        rearLeftDrive.setPower(0);
        rearRightDrive.setPower(0);
    }
}
