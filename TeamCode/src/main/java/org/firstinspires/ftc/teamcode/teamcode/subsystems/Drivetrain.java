package org.firstinspires.ftc.teamcode.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Drivetrain {
    private DcMotor flMotor;
    private DcMotor frMotor;
    private DcMotor blMotor;
    private DcMotor brMotor;
    private Servo lBrake;
    private Servo rBrake;
    public boolean isBraking;

    public Drivetrain(HardwareMap hardwareMap) {
        flMotor = hardwareMap.get(DcMotor.class, "fl");
        frMotor = hardwareMap.get(DcMotor.class, "fr");
        blMotor = hardwareMap.get(DcMotor.class, "bl");
        brMotor = hardwareMap.get(DcMotor.class, "br");

        lBrake = hardwareMap.get(Servo.class, "lBrake");
        rBrake = hardwareMap.get(Servo.class, "rBrake");

        isBraking = false;

        //change the direction of drivetrain motors so they're all facing the same way
        flMotor.setDirection(DcMotor.Direction.REVERSE);
        blMotor.setDirection(DcMotor.Direction.REVERSE);

        //make the motors brake
        flMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //reset encoder values
        flMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //run w/o velocity pid
        flMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        blMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setDrivetrainPower(double yPower, double xPower, double rPower) {
        //equations to move the robot
        flMotor.setPower(yPower + xPower + rPower);
        frMotor.setPower(yPower - xPower - rPower);
        blMotor.setPower(yPower - xPower + rPower);
        brMotor.setPower(yPower + xPower - rPower);
    }

    public void setDrivetrainPower(double yPower, double xPower, double rPower, double yPowerMultiplier, double xPowerMultiplier, double rPowerMultiplier) {
        yPower = yPower * yPowerMultiplier;
        xPower = xPower * xPowerMultiplier;
        rPower = rPower * rPowerMultiplier;

        //equations to move the robot
        flMotor.setPower(yPower + xPower + rPower);
        frMotor.setPower(yPower - xPower - rPower);
        blMotor.setPower(yPower - xPower + rPower);
        brMotor.setPower(yPower + xPower - rPower);
    }

    public void update() {
//        if (isBraking) {
//            lBrake.setPosition(0.5);
//            rBrake.setPosition(0.5);
//            isBraking = false;
//        } else {
//            lBrake.setPosition(0.5);
//            rBrake.setPosition(0.5);
//        }
    }

    public void printTelemetry(Telemetry telemetry) {
        telemetry.addData("fldrivetrain is ", flMotor.getCurrentPosition());
        telemetry.addData("frdrivetrain is ", frMotor.getCurrentPosition());
        telemetry.addData("bldrivetrain is ", blMotor.getCurrentPosition());
        telemetry.addData("brdrivetrain is ", brMotor.getCurrentPosition());
    }
}