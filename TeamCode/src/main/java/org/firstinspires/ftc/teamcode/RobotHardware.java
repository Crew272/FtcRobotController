package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

public class RobotHardware {
    public DcMotorEx frontLeft = null;
    public DcMotorEx backLeft = null;
    public DcMotorEx frontRight = null;
    public DcMotorEx backRight = null;
    public BNO055IMU imu = null;

    private HardwareMap hardwareMap = null;

    public void init(HardwareMap hwMap) {
        hardwareMap = hwMap;

        try {
            frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
            backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
            frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
            backRight = hardwareMap.get(DcMotorEx.class, "backRight");

            frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
            backLeft.setDirection(DcMotorEx.Direction.REVERSE);
            frontRight.setDirection(DcMotorEx.Direction.FORWARD);
            backRight.setDirection(DcMotorEx.Direction.FORWARD);

            frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

            frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

            frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

            imu = hardwareMap.get(BcMotorEx.class, "imu");
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.mode = BNO055IMU.SensorMode.IMU;
            imu.initialize(parameters);
        } catch (Exception e) {
            RobotLog.ee("RobotHardware", "Error initializing hardware: %s", e.getMessage());
        }
    }

    public double getHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

    public void testMotorDirections() {
        frontLeft.setPower(0.2);
        backLeft.setPower(0.2);
        frontRight.setPower(0.2);
        backRight.setPower(0.2);
    }
}