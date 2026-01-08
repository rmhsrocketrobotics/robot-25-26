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
    private final double zeroPowerTolerance = 0.075; // TODO test ts more

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
        //basic mecanum wheel kinematic equations
        double flPower = yPower + xPower + rPower;
        double frPower = yPower - xPower - rPower;
        double blPower = yPower - xPower + rPower;
        double brPower = yPower + xPower - rPower;

        //set power to zero if the power is low enough to activate braking
        if (Math.abs(flPower) < zeroPowerTolerance) {
            flPower = 0;
        }
        if (Math.abs(frPower) < zeroPowerTolerance) {
            frPower = 0;
        }
        if (Math.abs(blPower) < zeroPowerTolerance) {
            blPower = 0;
        }
        if (Math.abs(brPower) < zeroPowerTolerance) {
            brPower = 0;
        }

        flMotor.setPower(flPower);
        frMotor.setPower(frPower);
        blMotor.setPower(blPower);
        brMotor.setPower(brPower);
    }

    public void setDrivetrainPower(double yPower, double xPower, double rPower, double yPowerMultiplier, double xPowerMultiplier, double rPowerMultiplier) {
        yPower = yPower * yPowerMultiplier;
        xPower = xPower * xPowerMultiplier;
        rPower = rPower * rPowerMultiplier;

        setDrivetrainPower(yPower, xPower, rPower);
    }

    public void update() {
//        if (isBraking) {
//            lBrake.setPosition(0.35);
//            rBrake.setPosition(0.42);
//            isBraking = false;
//        } else {
//            lBrake.setPosition(0.6);
//            rBrake.setPosition(0.3);
//        }
    }

    public void printTelemetry(Telemetry telemetry) {
        telemetry.addData("fldrivetrain is ", flMotor.getCurrentPosition());
        telemetry.addData("frdrivetrain is ", frMotor.getCurrentPosition());
        telemetry.addData("bldrivetrain is ", blMotor.getCurrentPosition());
        telemetry.addData("brdrivetrain is ", brMotor.getCurrentPosition());
    }
}