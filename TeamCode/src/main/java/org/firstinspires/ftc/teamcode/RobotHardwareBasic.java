package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RobotHardwareBasic {
    /* Public OpMode members. */
    public DcMotorEx frontLeftDrive;
    public DcMotorEx frontRightDrive;
    public DcMotorEx rearLeftDrive;
    public DcMotorEx rearRightDrive;

    /* Local OpMode members. */
    HardwareMap hwMap;
    private ElapsedTime period = new ElapsedTime();

    // Constants for encoder counts
    public static final double COUNTS_PER_MOTOR_REV = 145.1;
    public static final double DRIVE_GEAR_REDUCTION = 2.0;
    public static final double WHEEL_DIAMETER_INCHES = 3.78;
    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    // Speed control
    private static final double MAX_DRIVE_SPEED = 0.6;
    private static final double MIN_DRIVE_SPEED = 0.2;

    /* Constructor */
    public RobotHardwareBasic() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors (note we're using DcMotorEx now)
        frontLeftDrive = hwMap.get(DcMotorEx.class, "frontLeftDrive");
        frontRightDrive = hwMap.get(DcMotorEx.class, "frontRightDrive");
        rearLeftDrive = hwMap.get(DcMotorEx.class, "rearLeftDrive");
        rearRightDrive = hwMap.get(DcMotorEx.class, "rearRightDrive");

        // Set motor directions
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        rearLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        rearRightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Set PIDF coefficients
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(5.0, 0.0, 0.0, 0.0);
        frontLeftDrive.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        frontRightDrive.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        rearLeftDrive.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        rearRightDrive.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        // Set all motors to zero power
        setPower(0);

        // Set all motors to run with encoders
        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set zero power behavior to BRAKE
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void moveToPosition(int targetPosition, double power) {
        // Reset encoders
        resetEncoders();

        // Set target position
        setTargetPosition(targetPosition);

        // Set mode to RUN_TO_POSITION
        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set power
        setPower(Math.abs(power));

        // Motors will automatically stop at target position
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        frontLeftDrive.setZeroPowerBehavior(behavior);
        frontRightDrive.setZeroPowerBehavior(behavior);
        rearLeftDrive.setZeroPowerBehavior(behavior);
        rearRightDrive.setZeroPowerBehavior(behavior);
    }

    public void resetEncoders() {
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setPower(double power) {
        // Limit power to MAX_DRIVE_SPEED
        power = Math.max(-MAX_DRIVE_SPEED, Math.min(power, MAX_DRIVE_SPEED));

        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(power);
        rearLeftDrive.setPower(power);
        rearRightDrive.setPower(power);
    }

    public void setIndividualPowers(double fl, double fr, double bl, double br) {
        frontLeftDrive.setPower(Math.max(-MAX_DRIVE_SPEED, Math.min(fl, MAX_DRIVE_SPEED)));
        frontRightDrive.setPower(Math.max(-MAX_DRIVE_SPEED, Math.min(fr, MAX_DRIVE_SPEED)));
        rearLeftDrive.setPower(Math.max(-MAX_DRIVE_SPEED, Math.min(bl, MAX_DRIVE_SPEED)));
        rearRightDrive.setPower(Math.max(-MAX_DRIVE_SPEED, Math.min(br, MAX_DRIVE_SPEED)));
    }

    public int getCurrentPosition() {
        return (frontLeftDrive.getCurrentPosition() + frontRightDrive.getCurrentPosition() +
                rearLeftDrive.getCurrentPosition() + rearRightDrive.getCurrentPosition()) / 4;
    }

    public boolean isBusy() {
        return frontLeftDrive.isBusy() && frontRightDrive.isBusy() &&
                rearLeftDrive.isBusy() && rearRightDrive.isBusy();
    }

    public void setTargetPosition(int position) {
        frontLeftDrive.setTargetPosition(position);
        frontRightDrive.setTargetPosition(position);
        rearLeftDrive.setTargetPosition(position);
        rearRightDrive.setTargetPosition(position);
    }

    public void setRunMode(DcMotor.RunMode mode) {
        frontLeftDrive.setMode(mode);
        frontRightDrive.setMode(mode);
        rearLeftDrive.setMode(mode);
        rearRightDrive.setMode(mode);
    }
}
