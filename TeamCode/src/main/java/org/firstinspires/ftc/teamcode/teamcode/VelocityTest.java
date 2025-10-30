package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp

public class VelocityTest extends LinearOpMode{

    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(2, 0, 3, 12);

    private VoltageSensor batteryVoltageSensor;
    DcMotorEx gecko;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        gecko = hardwareMap.get(DcMotorEx.class, "gecko");
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        setPIDFCoefficients(gecko, MOTOR_VELO_PID);

        waitForStart();

        gecko.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //gecko.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        gecko.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double timeThisLoop;
        double timeLastLoop = 0;
        double positionLastLoop = 0;

        while (opModeIsActive()){

            timeThisLoop = timer.seconds();

            double loopTime = timeThisLoop - timeLastLoop;
            double ticksTraveled = gecko.getCurrentPosition() - positionLastLoop;

            telemetry.addData("ticks traveled", ticksTraveled);
            telemetry.addData("loop time", loopTime);
            telemetry.addData("velocity ticks/s", ticksTraveled / loopTime);
            telemetry.addData("velocity meters/s", ticksToMeters(ticksTraveled) / loopTime);
            telemetry.update();

            double motorMetersPerSecond = ticksToMeters(ticksTraveled) / loopTime;

            timeLastLoop = timeThisLoop;
            positionLastLoop = gecko.getCurrentPosition();

            double targetMetersPerSecond = 0.3;

            double differenceMetersPerSecond = motorMetersPerSecond - targetMetersPerSecond;


            //gecko.setPower((-gamepad1.left_stick_y * 50) + 50);
             if (gamepad1.a) {
                 gecko.setVelocity(metersToTicks(.25));
             } else {
                 gecko.setVelocity(0);
             }

             if (gamepad1.b) {
                 gecko.setVelocity(metersToTicks(.5));
             } else {
                 gecko.setVelocity(0);
             }
        }
    }

    static double ticksToMeters(double ticks) {
        double revolutions = ticks / 537.7;
        double meters = revolutions * 0.072 * Math.PI;
        return meters;
    }

    static double metersToTicks(double meters) {
        double revolutions = meters / (0.072 * Math.PI);
        double ticks = revolutions * 537.7;
        return ticks;
    }

    static double ticksPerSecondToRPM(double ticksPerSec) {
        double ticksPerMinute = ticksPerSec * 60;
        double RPM = ticksPerMinute / 537.7;
        return RPM;
    }

    static double metersPerSecondToRPM(double metersPerSecond) {
        double metersPerMinute = metersPerSecond * 60;
        double RPM = metersPerMinute / (0.072 * Math.PI);
        return RPM;
    }

    private void setPIDFCoefficients(DcMotorEx motor, PIDFCoefficients coefficients) {
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d, coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        ));
    }
}