package org.firstinspires.ftc.teamcode.LastYearExampleCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


public abstract class MecanumDrivetrain extends LinearOpMode {
    private double xAcceleration;
    private double yAcceleration;
    private double rAcceleration;
    
    private double xDeceleration;
    private double yDeceleration;
    private double rDeceleration;
    
    private double xSpeed;
    private double ySpeed;
    private double rSpeed;
    
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;

    public IMU imu;

    public void initDrivetrain() {
        // theres def a better way to do this but idc
        frontLeft = hardwareMap.get(DcMotor.class, "fl");
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRight = hardwareMap.get(DcMotor.class, "fr");
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backLeft = hardwareMap.get(DcMotor.class, "bl");
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backRight = hardwareMap.get(DcMotor.class, "br");
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRight.setDirection(DcMotor.Direction.REVERSE); // do this for the motors so that they're all facing the right direction
        backRight.setDirection(DcMotor.Direction.REVERSE);
        
        // copy and pasted the following code (im pretty sure its magic)

        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        imu.resetYaw();
    }
    
    public void setRunToPosition() {
        frontLeft.setTargetPosition(0);
        frontRight.setTargetPosition(0);
        backLeft.setTargetPosition(0);
        backRight.setTargetPosition(0);
        
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //i just love copy and pasting
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //resetting encoders
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    
    public void setAccelerationValues(double xAcceleration, double yAcceleration, double rAcceleration, double xDeceleration, double yDeceleration, double rDeceleration) {
        this.xAcceleration = xAcceleration;
        this.yAcceleration = yAcceleration;
        this.rAcceleration = rAcceleration;
        
        this.xDeceleration = xDeceleration;
        this.yDeceleration = yDeceleration;
        this.rDeceleration = rDeceleration;
    }

    public double getRobotHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }


    public void setDrivetrainPowers(double x, double y, double r) { // r is short for rotation
        // magic ahh code lmao
        // you can do the calculations and stuff to derive these but they just move the robot so don't worry about it
        frontLeft.setPower(y + x + r);
        frontRight.setPower(y - x - r);
        backLeft.setPower(y - x + r);
        backRight.setPower(y + x - r);
    }

    public void setDrivetrainPowersAbsolute(double x, double y, double r) { // makes the robot move relative to the driver/field, not itself
        double robotHeading = getRobotHeading();

        double angleSine = Math.sin(- robotHeading);
        double angleCosine = Math.cos(- robotHeading);

        double rotatedX = x * angleCosine - y * angleSine;
        double rotatedY = x * angleSine + y * angleCosine;

        setDrivetrainPowers(rotatedX, rotatedY, r);
    }
    
    public double bringToZero(double value, double change) {
        //change should NOT be negative
        
        if (change >= Math.abs(value)) {
            return 0;
        }
        
        if (value > 0) {
            return value - change;
        }
        
        if (value < 0) {
            return value + change;
        }
        
        return 0;
    }
    
    public void accelerateDrivetrainPowers(double x, double y, double r, double loopTime) {
        xSpeed = bringToZero(xSpeed, xDeceleration * loopTime);
        ySpeed = bringToZero(ySpeed, yDeceleration * loopTime);
        rSpeed = bringToZero(rSpeed, rDeceleration * loopTime);
        
        xSpeed += x * xAcceleration * loopTime;
        ySpeed += y * yAcceleration * loopTime;
        rSpeed += r * rAcceleration * loopTime;
        
        xSpeed = bound(xSpeed, -1, 1);
        ySpeed = bound(ySpeed, -1, 1);
        rSpeed = bound(rSpeed, -1, 1);
        
        telemetry.addData("xSpeed", xSpeed);
        telemetry.addData("ySpeed", ySpeed);
        telemetry.addData("rSpeed", rSpeed);
        
        setDrivetrainPowers(xSpeed, ySpeed, rSpeed);
    }
    
    private static double bound(double num, double lowerBound, double upperBound) {
        return Math.max(Math.min(num, upperBound), lowerBound);
    }
}