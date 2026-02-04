package org.firstinspires.ftc.teamcode.teamcode.subsystems;

import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.teamcode.pidtuning.VelocityPIDFController;

public class Outtake {
    DcMotorEx outtake1;
    DcMotorEx outtake2;
    public Servo hoodServo;

    private VelocityPIDFController veloController;
    private double lastTargetTicksPerSecond;
    public double targetTicksPerSecond;

    private ElapsedTime sustainTimer; //keep the outtake running for a little while longer even after it's set to 0
    private final double sustainTime = 0.3;
    private double sustainTicksPerSecond;

    // the outtake must be going at +- this t/s for atTargetSpeed() to return true
    public double tolerance = 50;

    private final ElapsedTime veloTimer = new ElapsedTime();

    /// format: meters from goal -> outtake velocity (ticks per second)
    private InterpLUT outtakeVelocityLUT = new InterpLUT();

    /// format: meters from goal -> outtake angle (hood servo position)
    private InterpLUT outtakeAngleLUT = new InterpLUT();

    public Outtake(HardwareMap hardwareMap) {
        outtake1 = hardwareMap.get(DcMotorEx.class, "outtake1");
        outtake1.setDirection(DcMotor.Direction.REVERSE);
        outtake2 = hardwareMap.get(DcMotorEx.class, "outtake2");

        hoodServo = hardwareMap.get(Servo.class, "hoodServo");

        outtake1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtake2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        veloController = new VelocityPIDFController(0.006, 0, 0.00065, 0, 0);

        setLUTValues();

        lastTargetTicksPerSecond = 0;
        targetTicksPerSecond = 0;
    }

    public void init() {
        veloTimer.reset();

        sustainTimer = new ElapsedTime(100);
    }

    public void setOuttakePower(double power) {
        if (power < 0) {
            power = 0;
        }
        outtake1.setPower(power);
        outtake2.setPower(power);
    }

    public void printTelemetry(Telemetry telemetry) {
        double outtakeVelocity = outtake1.getVelocity();
        telemetry.addData("outtake target ticks/sec", targetTicksPerSecond);
        telemetry.addData("outtake ticks/sec", outtakeVelocity);
        telemetry.addData("atTargetSpeed()", atTargetSpeed());
        //telemetry.addData("outtake rpm", (outtakeVelocity * 60) / 28);
    }

    public boolean atTargetSpeed() {
        double currentTicksPerSecond = outtake1.getVelocity();
        double minTargetTicksPerSecond = targetTicksPerSecond - tolerance;
        double maxTargetTicksPerSecond = targetTicksPerSecond + tolerance;

        if (targetTicksPerSecond > 1650) {
            minTargetTicksPerSecond = 1650 - tolerance;
        }

        return (currentTicksPerSecond > minTargetTicksPerSecond) && (currentTicksPerSecond < maxTargetTicksPerSecond);
    }

    public void update() {
        // new code:
//        if (lastTargetTicksPerSecond > 0 && targetTicksPerSecond == 0) {
//            sustainTimer.reset();
//            sustainTicksPerSecond = lastTargetTicksPerSecond;
//        }
//        if (sustainTimer.seconds() < sustainTime) {
//            setOuttakeTicksPerSecond(sustainTicksPerSecond, sustainTicksPerSecond);
//        } else {
//            // ok so technically there's an issue here because when we switch from sustain back to real tps
//            // last target tps will be zero when it shouldn't be
//            // but i think that this might actually be fine
//            // bc the only thing last target tps is used for is kA
//            // which is zero
//            setOuttakeTicksPerSecond(targetTicksPerSecond, lastTargetTicksPerSecond);
//        }
//        lastTargetTicksPerSecond = targetTicksPerSecond;

        // old code:
        double targetAcceleration = (targetTicksPerSecond - lastTargetTicksPerSecond) / veloTimer.seconds();

        veloTimer.reset();
        lastTargetTicksPerSecond = targetTicksPerSecond;

        double motorPos = outtake1.getCurrentPosition();
        double motorVelo = outtake1.getVelocity();

        double power = veloController.update(motorPos, motorVelo, targetTicksPerSecond, targetAcceleration);
        setOuttakePower(power);
    }

//    private void setOuttakeTicksPerSecond(double targetTicksPerSecond, double lastTargetTicksPerSecond) {
//        double targetAcceleration = (targetTicksPerSecond - lastTargetTicksPerSecond) / veloTimer.seconds();
//
//        veloTimer.reset();
//
//        double motorPos = outtake1.getCurrentPosition();
//        double motorVelo = outtake1.getVelocity();
//
//        double power = veloController.update(motorPos, motorVelo, targetTicksPerSecond, targetAcceleration);
//        setOuttakePower(power);
//    }

    private void setLUTValues() {
        outtakeVelocityLUT.add(0.5, 1000);
        outtakeAngleLUT.add(0.5, 0);

        outtakeVelocityLUT.add(0.75, 1050);
        outtakeAngleLUT.add(0.75, 0);

        outtakeVelocityLUT.add(1, 1100);
        outtakeAngleLUT.add(1, 0.25);

        outtakeVelocityLUT.add(1.25, 1150);
        outtakeAngleLUT.add(1.25, 0.25);

        outtakeVelocityLUT.add(1.5, 1200);
        outtakeAngleLUT.add(1.5, 0.3);

        outtakeVelocityLUT.add(1.75, 1250);
        outtakeAngleLUT.add(1.75, 0.3);

        outtakeVelocityLUT.add(2, 1300);
        outtakeAngleLUT.add(2, 0.4);

        outtakeVelocityLUT.add(2.25, 1350);
        outtakeAngleLUT.add(2.25, 0.4);

        outtakeVelocityLUT.add(2.5, 1350);
        outtakeAngleLUT.add(2.5, 0.4);

        outtakeVelocityLUT.add(2.9, 1450);
        outtakeAngleLUT.add(2.9, 0.4);

        outtakeVelocityLUT.add(3.1, 1500);
        outtakeAngleLUT.add(3.1, 0.4);

        outtakeVelocityLUT.createLUT();
        outtakeAngleLUT.createLUT();
    }

    /// set the speed of the outtake and the angle of the hood based the distance to the goal
    /// this method works based off of empirical measurements
    public void setOuttakeVelocityAndHoodAngle(double metersFromGoal) {
        // clamp this to be a slightly smaller range than the values you entered into the LUT
        // this is bc InterpLUT throws an error if input <= mX.get(0)
        metersFromGoal = CustomMath.clamp(metersFromGoal, 0.5 + 0.0001, 3.1 - 0.0001);

        targetTicksPerSecond = outtakeVelocityLUT.get(metersFromGoal);
        hoodServo.setPosition(outtakeAngleLUT.get(metersFromGoal));
    }
}