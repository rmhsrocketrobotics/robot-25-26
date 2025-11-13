package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp

public class MotorTest extends LinearOpMode{

    DcMotorEx gecko;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        gecko = hardwareMap.get(DcMotorEx.class, "outtake");

        gecko.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();

        double timeThisLoop = 0;
        double timeLastLoop = 0;
        double positionLastLoop = 0;

        while (opModeIsActive()){

            if (timer.seconds() - timeLastLoop > 0.01) {
                timeThisLoop = timer.seconds();

                double loopTime = timeThisLoop - timeLastLoop;
                double ticksTraveled = gecko.getCurrentPosition() - positionLastLoop;

                telemetry.addData("ticks traveled", ticksTraveled);
                telemetry.addData("loop time", loopTime);
                telemetry.addData("(our measurements) velocity ticks/s", ticksTraveled / loopTime);
                telemetry.addData("(motor's measurements) velocity ticks/s", gecko.getVelocity());
                //telemetry.addData("velocity meters/s", ticksToMeters(ticksTraveled) / loopTime);
                telemetry.update();

                double motorMetersPerSecond = ticksToMeters(ticksTraveled) / loopTime;

                timeLastLoop = timeThisLoop;
                positionLastLoop = gecko.getCurrentPosition();
            }


            //gecko.setPower(-gamepad1.left_stick_y);
            if (gamepad1.b) {
                gecko.setPower(-1);
            } else if (gamepad1.a) {
                gecko.setPower(-0.75);
            } else if (gamepad1.x) {
                gecko.setPower(-0.5);
            } else if (gamepad1.y) {
                gecko.setPower(-0.25);
            } else {
                gecko.setPower(0);
            }
        }
    }

    static double ticksToMeters(double ticks) {
        double revolutions = ticks / 537.7;
        double meters = revolutions * 0.096 * Math.PI;
        return meters;
    }

    static double metersToTicks(double meters) {
        double revolutions = meters / (0.096 * Math.PI);
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
        double RPM = metersPerMinute / (0.096 * Math.PI);
        return RPM;
    }
}