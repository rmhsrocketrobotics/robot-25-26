package org.firstinspires.ftc.teamcode.LastYearExampleCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
public abstract class MecanumDrivetrain3 extends LinearOpMode {
    public PIDController flPID; // each of these control one wheel
    public PIDController frPID;
    public PIDController blPID;
    public PIDController brPID;

    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;

    public void initDrivetrain() {
        flPID = new PIDController(0.015, 0, 0.0001, 20, 0.35);
        frPID = new PIDController(0.015, 0, 0.0001, 20, 0.35);
        blPID = new PIDController(0.015, 0, 0.0001, 20, 0.35);
        brPID = new PIDController(0.015, 0, 0.0001, 20, 0.35);

        // theres def a better way to do this but idc
        frontLeft = hardwareMap.get(DcMotor.class, "fl");
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        frontRight = hardwareMap.get(DcMotor.class, "fr");
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        backLeft = hardwareMap.get(DcMotor.class, "bl");
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        backRight = hardwareMap.get(DcMotor.class, "br");
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        frontLeft.setDirection(DcMotor.Direction.REVERSE); // do this for the motors so that they're all facing the right direction
        backLeft.setDirection(DcMotor.Direction.REVERSE);
    }

    public void setDrivetrainPowers(double x, double y, double r) { // r is short for rotation
        frontLeft.setPower(y + x + r);
        frontRight.setPower(y - x - r);
        backLeft.setPower(y - x + r);
        backRight.setPower(y + x - r);
    }
    
    public boolean motorsAreDone() {
        return ((flPID.completed && frPID.completed) && (blPID.completed && brPID.completed));
    }
    
    //treat this like a graph, positive x is right, positive y is up (forward)
    public void move(double xTicks, double yTicks, double degrees, double speed) {}
    
    public void move(double xTicks, double yTicks) {}
    
    public void turn(double degrees) {}
}
