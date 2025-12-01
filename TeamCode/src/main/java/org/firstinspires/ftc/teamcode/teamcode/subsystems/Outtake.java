package org.firstinspires.ftc.teamcode.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Outtake {
    DcMotorEx outtake1;
    DcMotorEx outtake2;
    public Servo hoodServo;
    PIDFCoefficients MOTOR_VELO_PID;
    VoltageSensor batteryVoltageSensor;
    public double targetTicksPerSecond;

    // the outtake must be going at +- this t/s for atTargetSpeed() to return true
    public double tolerance = 300;
    public Outtake(HardwareMap hardwareMap) {
        outtake1 = hardwareMap.get(DcMotorEx.class, "outtake1");
        outtake1.setDirection(DcMotor.Direction.REVERSE);
        outtake2 = hardwareMap.get(DcMotorEx.class, "outtake2");
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");

        MOTOR_VELO_PID = new PIDFCoefficients(25, 0, 0, 19);

        MotorConfigurationType motorConfigurationType = outtake1.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        outtake1.setMotorType(motorConfigurationType);
        outtake2.setMotorType(motorConfigurationType);

        outtake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        setPIDFCoefficients(outtake1, MOTOR_VELO_PID);
        setPIDFCoefficients(outtake2, MOTOR_VELO_PID);

        targetTicksPerSecond = 0;
    }

    public void setOuttakePower(double power) {
        outtake1.setPower(-power);
        outtake2.setPower(-power);
    }

    private void setOuttakeVelocityTPS(double ticksPerSecond) {
        outtake1.setVelocity(ticksPerSecond);
        outtake2.setVelocity(ticksPerSecond);
    }

    public void printTelemetry(Telemetry telemetry) {
        double outtakeVelocity = outtake1.getVelocity();
        telemetry.addData("outtake ticks/sec", outtakeVelocity);
        telemetry.addData("outtake rev/min", (outtakeVelocity * 60) / 28);
    }

    private void setPIDFCoefficients(DcMotorEx motor, PIDFCoefficients coefficients) {
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d, coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        ));
    }

    private boolean aboveMaxPowerThreshold() {
        return targetTicksPerSecond > 1400-75;
    }

    public boolean atTargetSpeed() {
        if (aboveMaxPowerThreshold()) {
            return outtake1.getVelocity() > 1600;
        }
        double minTargetTicksPerSecond = targetTicksPerSecond - tolerance;
        double maxTargetTicksPerSecond = targetTicksPerSecond + tolerance;
        double currentTicksPerSecond = outtake1.getVelocity();
        return (currentTicksPerSecond > minTargetTicksPerSecond) && (currentTicksPerSecond < maxTargetTicksPerSecond);
    }

    public void update() {
        if (aboveMaxPowerThreshold()) {
            setOuttakePower(-0.9);
        } else {
            setOuttakeVelocityTPS(targetTicksPerSecond);
        }
        //setOuttakeVelocityTPS(targetTicksPerSecond);
    }

    public void setOuttakeAndHoodToVelocity(Velocity velocity) {
        // change meters per second into ticks per second
        double metersPerSecond = CustomMath.clamp(velocity.speed, 4, 6);
        targetTicksPerSecond = metersPerSecondToTicksPerSecond(metersPerSecond);

        setHoodServoToAngle(velocity.direction);
    }

    private void setHoodServoToAngle(double degrees) {
        // all angles are in degrees above the horizon
        double maxDegrees = 60;
        double minDegrees = 45;
        double maxPosition = 0.5;
        double minPosition = 0;

        degrees = CustomMath.clamp(degrees, minDegrees, maxDegrees);
        double percentRaised = 1 - ( (degrees - minDegrees) / (maxDegrees - minDegrees) );
        double position = (percentRaised * (maxPosition - minPosition)) + minPosition;

        hoodServo.setPosition(position);
    }

    private double metersPerSecondToTicksPerSecond(double metersPerSecond) {
        // this calculation is based on https://www.desmos.com/calculator/ye3y2vp7c2
        double m = 262.7892;
        double b = -64.05544 - 75;
        return (m * metersPerSecond) + b;
    }
}