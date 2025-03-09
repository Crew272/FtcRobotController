package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="ForwardOneFoot", group="Linear Opmode")
public class ForwardOneFoot extends LinearOpMode {

    // Define hardware
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    // Constants
    private static final double TICKS_PER_REV = 537.6; // For GoBilda 5202 motors with 537.6 ticks/rev
    private static final double WHEEL_DIAMETER_INCHES = 4.0; // Diameter of the wheel in inches
    private static final double INCHES_PER_REV = Math.PI * WHEEL_DIAMETER_INCHES;
    private static final double TICKS_PER_INCH = TICKS_PER_REV / INCHES_PER_REV;
    private static final double TARGET_DISTANCE_INCHES = 12.0; // 1 foot = 12 inches

    @Override
    public void runOpMode() {
        // Initialize hardware
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        // Set motor direction (depends on robot wiring)
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // Reset and initialize encoders
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set motors to run using encoders
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the start button
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        // Set target position (distance in ticks)
        int targetTicks = (int) (TARGET_DISTANCE_INCHES * TICKS_PER_INCH);

        // Set motors to target speed
        frontLeft.setPower(0.5);
        frontRight.setPower(0.5);
        backLeft.setPower(0.5);
        backRight.setPower(0.5);

        // Move until the target distance is reached
        while (opModeIsActive() &&
                Math.abs(frontLeft.getCurrentPosition()) < targetTicks &&
                Math.abs(frontRight.getCurrentPosition()) < targetTicks &&
                Math.abs(backLeft.getCurrentPosition()) < targetTicks &&
                Math.abs(backRight.getCurrentPosition()) < targetTicks) {

            telemetry.addData("Target Ticks", targetTicks);
            telemetry.addData("Front Left Ticks", frontLeft.getCurrentPosition());
            telemetry.addData("Front Right Ticks", frontRight.getCurrentPosition());
            telemetry.addData("Back Left Ticks", backLeft.getCurrentPosition());
            telemetry.addData("Back Right Ticks", backRight.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motors
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        // Reset encoders again
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("Status", "Motion Completed");
        telemetry.update();
    }
}

