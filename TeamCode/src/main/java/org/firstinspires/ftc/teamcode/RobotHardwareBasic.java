package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotHardwareBasic {
    // Motor declarations with your specific names
    public DcMotor frontLeftDrive = null;
    public DcMotor frontRightDrive = null;
    public DcMotor rearLeftDrive = null;
    public DcMotor rearRightDrive = null;

    // Constants for encoder counts
    public static final double COUNTS_PER_MOTOR_REV = 145.1; // GoBilda 5202 series 1150 RPM motor
    public static final double DRIVE_GEAR_REDUCTION = 2.0;    // The motor gear is smaller then the output gear
    public static final double WHEEL_DIAMETER_INCHES = 3.78;  // GoBilda Mecanum wheel diameter
    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    // Constants for rotation
    public static final double ROBOT_DIAMETER_INCHES = 18.0; // Measure your robot's turning diameter
    public static final double COUNTS_PER_DEGREE =
            (ROBOT_DIAMETER_INCHES * Math.PI * COUNTS_PER_INCH) / 360.0;

    // Constructor
    public RobotHardwareBasic(HardwareMap hwMap) {
        initialize(hwMap);
    }

    private void initialize(HardwareMap hwMap) {
        // Initialize motors with your specific names
        frontLeftDrive = hwMap.get(DcMotor.class, "frontLeftDrive");
        frontRightDrive = hwMap.get(DcMotor.class, "frontRightDrive");
        rearLeftDrive = hwMap.get(DcMotor.class, "rearLeftDrive");
        rearRightDrive = hwMap.get(DcMotor.class, "rearRightDrive");

        // Set motor directions (adjust these based on your robot's configuration)
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        rearLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        rearRightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Set zero power behavior
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Stop and reset encoders
        resetEncoders();
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

    public void setTargetPosition(int counts) {
        frontLeftDrive.setTargetPosition(counts);
        frontRightDrive.setTargetPosition(counts);
        rearLeftDrive.setTargetPosition(counts);
        rearRightDrive.setTargetPosition(counts);
    }

    public void setRunMode(DcMotor.RunMode mode) {
        frontLeftDrive.setMode(mode);
        frontRightDrive.setMode(mode);
        rearLeftDrive.setMode(mode);
        rearRightDrive.setMode(mode);
    }

    public void setPower(double power) {
        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(power);
        rearLeftDrive.setPower(power);
        rearRightDrive.setPower(power);
    }

    public void setIndividualPowers(double fl, double fr, double bl, double br) {
        frontLeftDrive.setPower(fl);
        frontRightDrive.setPower(fr);
        rearLeftDrive.setPower(bl);
        rearRightDrive.setPower(br);
    }

    public void setRotationPowers(double power) {
        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(-power);
        rearLeftDrive.setPower(power);
        rearRightDrive.setPower(-power);
    }

    public boolean isBusy() {
        return frontLeftDrive.isBusy() && frontRightDrive.isBusy() &&
                rearLeftDrive.isBusy() && rearRightDrive.isBusy();
    }

    public int getCurrentPosition() {
        // Returns average of all encoders
        return (frontLeftDrive.getCurrentPosition() + frontRightDrive.getCurrentPosition() +
                rearLeftDrive.getCurrentPosition() + rearRightDrive.getCurrentPosition()) / 4;
    }
}

