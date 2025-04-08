package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


public class RobotHardwareBasic {

    // Motor declarations
    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx rearLeft;
    public DcMotorEx rearRight;

    // IMU declaration
    public IMU imu;

    // Wheel diameter in mm
    private final double WHEEL_DIAMETER_MM = 100.0;

    // Encoder ticks per revolution for the AndyMark NeverRest 20
    private final double TICKS_PER_REV = 537.7;

    // Robot dimensions (distance between wheel centers) in mm
    private final double TRACK_WIDTH_MM = 315.0; // Center to center distance between side wheels
    private final double WHEEL_BASE_MM = 315.0; // Center to center distance between front and back wheels

    // Current robot position
    private double currentX = 0;
    private double currentY = 0;
    private double currentHeading = 0;

    public void init(HardwareMap hwMap) {
        // Initialize motors
        frontLeft = hwMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hwMap.get(DcMotorEx.class, "frontRight");
        rearLeft = hwMap.get(DcMotorEx.class, "rearLeft");
        rearRight = hwMap.get(DcMotorEx.class, "rearRight");

        // Set motor directions
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        rearLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        rearRight.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set motor modes
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set zero power behavior
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize IMU (Rev Expansion Hub)
        imu = hwMap.get(IMU.class, "imu");
        // Adjust the orientation parameters based on how your hub is mounted
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);
    }

    public void resetPosAndIMU() {
        // Reset motor encoders
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Reset IMU
        imu.resetYaw();

        // Reset current pose
        currentX = 0;
        currentY = 0;
        currentHeading = 0;
    }

    public void update() {
        // Get encoder values
        int flPos = frontLeft.getCurrentPosition();
        int frPos = frontRight.getCurrentPosition();
        int rlPos = rearLeft.getCurrentPosition();
        int rrPos = rearRight.getCurrentPosition();

        // Calculate distance traveled by each wheel
        double flDist = encoderTicksToDistance(flPos);
        double frDist = encoderTicksToDistance(frPos);
        double rlDist = encoderTicksToDistance(rlPos);
        double rrDist = encoderTicksToDistance(rrPos);

        // Calculate average forward/backward movement
        double forwardBackward = (flDist + frDist + rlDist + rrDist) / 4.0;

        // Calculate average strafing movement
        double strafe = (frDist + rlDist - flDist - rrDist) / 4.0;

        // Calculate change in heading
        double deltaHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Update current pose
        currentX = currentX + forwardBackward * Math.cos(currentHeading) - strafe * Math.sin(currentHeading);
        currentY = currentY + forwardBackward * Math.sin(currentHeading) + strafe * Math.cos(currentHeading);
        currentHeading = deltaHeading;
    }

    // Helper method to convert encoder ticks to distance (mm)
    private double encoderTicksToDistance(int ticks) {
        double wheelCircumference = Math.PI * WHEEL_DIAMETER_MM;
        double distancePerTick = wheelCircumference / TICKS_PER_REV;
        return ticks * distancePerTick;
    }

    public double getCurrentX() {
        return currentX;
    }
    public double getCurrentY() {
        return currentY;
    }
    public double getCurrentHeading() {
        return currentHeading;
    }
}