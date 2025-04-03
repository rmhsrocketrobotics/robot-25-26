package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


public abstract class MecanumDrivetrain2 extends LinearOpMode {
    public PIDController xController; // controls left-right movement
    public PIDController yController; // controls backward-forward movement
    public PIDController rController; // controls rotation

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    public void initDrivetrain() {
        xController = new PIDController(0.01, 0, 0, 25, 0.2);//new PIDController(0.01, 0, 0, 50, 0.075);
        yController = new PIDController(0.01, 0, 0, 25, 0.2);//new PIDController(0.01, 0, 0, 50, 0.075);
        rController = new PIDController(0.01, 0, 0, 20, 0.6); // insert settings from rotation thingy

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

        frontLeft.setDirection(DcMotor.Direction.REVERSE); // do this for the motors so that they're all facing the right direction
        backLeft.setDirection(DcMotor.Direction.REVERSE);
    }

    public double getXPosition() {
        return ( (frontLeft.getCurrentPosition()) + (-frontRight.getCurrentPosition()) + (-backLeft.getCurrentPosition()) + (backRight.getCurrentPosition()) ) / 4;
    }

    public double getYPosition() {
        return ( frontLeft.getCurrentPosition() + frontRight.getCurrentPosition() + backLeft.getCurrentPosition() + backRight.getCurrentPosition() ) / 4;
    }

    public double getRotation() {
        return ( (frontLeft.getCurrentPosition()) + (-frontRight.getCurrentPosition()) + (backLeft.getCurrentPosition()) + (-backRight.getCurrentPosition()) ) / 4;
    }

    public void setDrivetrainPowers(double x, double y, double r) { // r is short for rotation
        frontLeft.setPower(y + x + r);
        frontRight.setPower(y - x - r);
        backLeft.setPower(y - x + r);
        backRight.setPower(y + x - r);
    }
    
    public void move(double xTicks, double yTicks) {
        move(xTicks, yTicks, 0.5);
    }
    
    //treat this like a graph, positive x is right, positive y is up (forward)
    public void move(double xTicks, double yTicks, double speed) { // lets you set a position for the motors and let the pid do the work
        xTicks = -xTicks;
        yTicks = -yTicks;
        
        double startXPosition = getXPosition();
        double startYPosition = getYPosition();

        double currentXPosition = getXPosition() - startXPosition;
        double currentYPosition = getYPosition() - startYPosition;

        double targetXPosition = xTicks;
        double targetYPosition = yTicks;

        while (opModeIsActive() && (!xController.completed || !yController.completed)) {
            currentXPosition = getXPosition() - startXPosition;
            currentYPosition = getYPosition() - startYPosition;

            double xPower = xController.update(currentXPosition, targetXPosition);
            double yPower = yController.update(currentYPosition, targetYPosition);

            setDrivetrainPowers(xPower * speed, yPower * speed, 0);
            
            telemetry.addData("start y", startYPosition);
            telemetry.addData("current y", currentYPosition);
            telemetry.addData("target y", targetYPosition);
            telemetry.update();
        }
        setDrivetrainPowers(0, 0, 0);
        xController.reset();
        yController.reset();
    }
    
    public void turn(double degrees) {
        turn(degrees, 0.75);
    }

    public void turn(double degrees, double speed) { // lets you set a position for the motors and let the pid do the work
        degrees = -degrees;
        
        double startDegrees = getRotation();

        double currentDegrees = getRotation() - startDegrees;

        double targetDegrees = degrees * 12.5; //insert conversion factor here

        while (opModeIsActive() && (!rController.completed)) {
            currentDegrees = getRotation() - startDegrees;

            double rPower = rController.update(currentDegrees, targetDegrees);

            setDrivetrainPowers(0, 0, rPower * speed);
            
            telemetry.addData("rController.completed", rController.completed);
            telemetry.addData("start", startDegrees);
            telemetry.addData("current", currentDegrees);
            telemetry.addData("target", targetDegrees);
            telemetry.update();
        }
        setDrivetrainPowers(0, 0, 0);
        rController.reset();
    }
}
