package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.CRServo;

public class RobotHardware {
    // Drivetrain Motors (Mecanum Wheels)
    public DcMotor frontLeftDrive;
    public DcMotor frontRightDrive;
    public DcMotor rearLeftDrive;
    public DcMotor rearRightDrive;

    // Grabber 1 Components
    public DcMotor grabber1LiftMotor;
    public Servo arm1RotationServo;
    public Servo claw1GrabServo;

    // Touch Sensor for LiftMotor Zero Position
    public TouchSensor grabber1DownSwitch;

    // Grabber 2 Components
    // not using //public Servo slideLeftServo;
    public Servo slideRightServo;
    public CRServo intake1;
    public Servo upAndDownServo; // Added servo for up and down movement



    // Hardware Map
    private HardwareMap hardwareMap;

    // Encoder limit for the lift
    public static final int LIFT_MAX_HEIGHT_TICKS = -4000;

    // Constructor
    public RobotHardware(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        initializeHardware();
    }

    private void initializeHardware() {
        // Drivetrain Motors Initialization
        frontLeftDrive = hardwareMap.dcMotor.get("frontLeftDrive");
        frontRightDrive = hardwareMap.dcMotor.get("frontRightDrive");
        rearLeftDrive = hardwareMap.dcMotor.get("rearLeftDrive");
        rearRightDrive = hardwareMap.dcMotor.get("rearRightDrive");

        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rearLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Grabber 1 Initialization
        // Initialize the lift motor
        grabber1LiftMotor = hardwareMap.dcMotor.get("grabber1LiftMotor");
        // Touch Sensor Initialization
        grabber1DownSwitch = hardwareMap.touchSensor.get("grabber1DownSwitch");
        arm1RotationServo = hardwareMap.servo.get("arm1RotationServo");
        claw1GrabServo = hardwareMap.servo.get("claw1GrabServo");
        // Configure motor
        grabber1LiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        grabber1LiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        grabber1LiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Grabber 2 Initialization
        slideRightServo = hardwareMap.servo.get("slideRightServo");
        intake1 = hardwareMap.get(CRServo.class, "intake1");
        upAndDownServo = hardwareMap.servo.get("upAndDownServo");  // New servo
    }

    // Returns whether the lift is at the lowest position
    public boolean isLiftAtBottom() {
        return grabber1DownSwitch.isPressed();
    }
}