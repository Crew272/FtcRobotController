package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="Calibrate", group="chad")
public class Calibrate extends LinearOpMode {
    //
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor rearLeftDrive;
    DcMotor rearRightDrive;
    //Calculate encoder conversion
    Integer cpr = 28; //counts per rotation
    Integer gearratio = 40;
    Double diameter = 4.125;
    Double cpi = 11.28; //(cpr * gearratio) / (Math.PI * diameter); //counts per inch -> counts per rotation / circumference
    Double bias = 0.8;//adjust until your robot goes 20 inches
    //
    Double conversion = cpi * bias;
    //
    public void runOpMode() {
        //
        frontLeftDrive = hardwareMap.dcMotor.get("frontLeftDrive");
        frontRightDrive = hardwareMap.dcMotor.get("frontRightDrive");
        rearLeftDrive = hardwareMap.dcMotor.get("rearLeftDrive");
        rearRightDrive = hardwareMap.dcMotor.get("rearRightDrive");
        frontRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);//If your robot goes backward, switch this from right to left
        rearRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);//If your robot goes backward, switch this from right to left
        //
        waitForStartify();
        //
        moveToPosition(20, .2);//Don't change this line, unless you want to calibrate with different speeds
        //
    }
    //
    /*
    This function's purpose is simply to drive forward or backward.
    To drive backward, simply make the inches input negative.
     */
    public void moveToPosition(double inches, double speed) {
        //
        if (inches < 5) {
            int move = (int) (Math.round(inches * conversion));
            //
            frontLeftDrive.setTargetPosition(frontLeftDrive.getCurrentPosition() + move);
            frontRightDrive.setTargetPosition(frontRightDrive.getCurrentPosition() + move);
            rearLeftDrive.setTargetPosition(rearLeftDrive.getCurrentPosition() + move);
            rearRightDrive.setTargetPosition(rearRightDrive.getCurrentPosition() + move);
            //
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rearLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rearRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //
            frontLeftDrive.setPower(speed);
            frontRightDrive.setPower(speed);
            rearLeftDrive.setPower(speed);
            rearRightDrive.setPower(speed);
            //
            while (frontLeftDrive.isBusy() && frontRightDrive.isBusy() && rearLeftDrive.isBusy() && rearRightDrive.isBusy()) {
            }
            frontLeftDrive.setPower(0);
            frontRightDrive.setPower(0);
            rearLeftDrive.setPower(0);
            rearRightDrive.setPower(0);
        } else {
            int move1 = (int) (Math.round((inches - 5) * conversion));
            int movefl2 = frontLeftDrive.getCurrentPosition() + (int) (Math.round(inches * conversion));
            int movefr2 = frontRightDrive.getCurrentPosition() + (int) (Math.round(inches * conversion));
            int movebl2 = rearLeftDrive.getCurrentPosition() + (int) (Math.round(inches * conversion));
            int movebr2 = rearRightDrive.getCurrentPosition() + (int) (Math.round(inches * conversion));
            //
            frontLeftDrive.setTargetPosition(frontLeftDrive.getCurrentPosition() + move1);
            frontRightDrive.setTargetPosition(frontRightDrive.getCurrentPosition() + move1);
            rearLeftDrive.setTargetPosition(rearLeftDrive.getCurrentPosition() + move1);
            rearRightDrive.setTargetPosition(rearRightDrive.getCurrentPosition() + move1);
            //
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rearLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rearRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //
            frontLeftDrive.setPower(speed);
            frontRightDrive.setPower(speed);
            rearLeftDrive.setPower(speed);
            rearRightDrive.setPower(speed);
            //
            while (frontLeftDrive.isBusy() && frontRightDrive.isBusy() && rearLeftDrive.isBusy() && rearRightDrive.isBusy()) {
            }
            //
            frontLeftDrive.setTargetPosition(movefl2);
            frontRightDrive.setTargetPosition(movefr2);
            rearLeftDrive.setTargetPosition(movebl2);
            rearRightDrive.setTargetPosition(movebr2);
            //
            frontLeftDrive.setPower(.1);
            frontRightDrive.setPower(.1);
            rearLeftDrive.setPower(.1);
            rearRightDrive.setPower(.1);
            //
            while (frontLeftDrive.isBusy() && frontRightDrive.isBusy() && rearLeftDrive.isBusy() && rearRightDrive.isBusy()) {
            }
            frontLeftDrive.setPower(0);
            frontRightDrive.setPower(0);
            rearLeftDrive.setPower(0);
            rearRightDrive.setPower(0);
        }
        return;
    }
    /*
    A tradition within the Thunder Pengwins code, we always start programs with waitForStartify,
    our way of adding personality to our programs.
     */
    public void waitForStartify() {
        waitForStart();
    }
}
