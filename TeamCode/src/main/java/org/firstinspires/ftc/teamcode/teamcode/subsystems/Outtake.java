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

    // the outtake must be going at +- this t/s for atTargetSpeed() to return true
    public double tolerance = 40;

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

        veloController = new VelocityPIDFController(0.004, 0, 0.00065, 0, 0);

        setLUTValues();

        lastTargetTicksPerSecond = 0;
        targetTicksPerSecond = 0;
    }

    public void init() {
        veloTimer.reset();
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
        double targetAcceleration = (targetTicksPerSecond - lastTargetTicksPerSecond) / veloTimer.seconds();

        veloTimer.reset();
        lastTargetTicksPerSecond = targetTicksPerSecond;

        double motorPos = outtake1.getCurrentPosition();
        double motorVelo = outtake1.getVelocity();

        double power = veloController.update(motorPos, motorVelo, targetTicksPerSecond, targetAcceleration);
        setOuttakePower(power);
    }

    private void setLUTValues() {
        outtakeVelocityLUT.add(0.5, 1000);
        outtakeAngleLUT.add(0.5, 0);

        outtakeVelocityLUT.add(0.75, 1050);
        outtakeAngleLUT.add(0.75, 0);

        outtakeVelocityLUT.add(1, 1150);
        outtakeAngleLUT.add(1, 0.25);

        outtakeVelocityLUT.add(1.25, 1200);
        outtakeAngleLUT.add(1.25, 0.25);

        outtakeVelocityLUT.add(1.5, 1200);
        outtakeAngleLUT.add(1.5, 0.4);

        outtakeVelocityLUT.add(1.75, 1300);
        outtakeAngleLUT.add(1.75, 0.4);

        outtakeVelocityLUT.add(2, 1350);
        outtakeAngleLUT.add(2, 0.4);

        outtakeVelocityLUT.add(2.25, 1400);
        outtakeAngleLUT.add(2.25, 0.4);

        outtakeVelocityLUT.add(2.5, 1400);
        outtakeAngleLUT.add(2.5, 0.4);

        outtakeVelocityLUT.add(2.9, 1500);
        outtakeAngleLUT.add(2.9, 0.4);

        outtakeVelocityLUT.createLUT();
        outtakeAngleLUT.createLUT();
    }

    /// set the speed of the outtake and the angle of the hood based the distance to the goal
    /// this method works based off of empirical measurements
    public void setOuttakeVelocityAndHoodAngle(double metersFromGoal) {
        // clamp this to be a slightly smaller range than the values you entered into the LUT
        // this is bc InterpLUT throws an error if input <= mX.get(0)
        metersFromGoal = CustomMath.clamp(metersFromGoal, 0.75 + 0.0001, 2.5 - 0.0001);

        targetTicksPerSecond = outtakeVelocityLUT.get(metersFromGoal);
        hoodServo.setPosition(outtakeAngleLUT.get(metersFromGoal));
    }
}