package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp

public class MotorTest extends LinearOpMode{

    DcMotorEx outtake1;
    DcMotorEx outtake2;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        outtake1 = hardwareMap.get(DcMotorEx.class, "outtake1");
        outtake2 = hardwareMap.get(DcMotorEx.class, "outtake2");

        outtake1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtake2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtake1.setDirection(DcMotor.Direction.REVERSE);


        waitForStart();

        double timeThisLoop = 0;
        double timeLastLoop = 0;
        double positionLastLoop = 0;

        while (opModeIsActive()){

//            if (timer.seconds() - timeLastLoop > 0.01) {
//                timeThisLoop = timer.seconds();
//
//                double loopTime = timeThisLoop - timeLastLoop;
//                double ticksTraveled = gecko.getCurrentPosition() - positionLastLoop;
//
//                telemetry.addData("ticks traveled", ticksTraveled);
//                telemetry.addData("loop time", loopTime);
//                telemetry.addData("(our measurements) velocity ticks/s", ticksTraveled / loopTime);
//                telemetry.addData("(motor's measurements) velocity ticks/s", gecko.getVelocity());
//                //telemetry.addData("velocity meters/s", ticksToMeters(ticksTraveled) / loopTime);
//                telemetry.update();
//
//                double motorMetersPerSecond = ticksToMeters(ticksTraveled) / loopTime;
//
//                timeLastLoop = timeThisLoop;
//                positionLastLoop = gecko.getCurrentPosition();
//            }

//            outtake1.setPower(gamepad1.left_stick_y * 0.5);
//            outtake2.setPower(gamepad1.right_stick_y * 0.5);

            //gecko.setPower(-gamepad1.left_stick_y);
            if (gamepad1.b) {
                outtake1.setPower(1);
                outtake2.setPower(1);
            } else if (gamepad1.a) {
                outtake1.setPower(0.75);
                outtake2.setPower(0.75);
            } else if (gamepad1.x) {
                outtake1.setPower(0.5);
                outtake2.setPower(0.5);
            } else if (gamepad1.y) {
                outtake1.setPower(0.25);
                outtake2.setPower(0.25);
            } else {
                outtake1.setPower(0);
                outtake2.setPower(0);
            }

            telemetry.addData("outtake1 speed rpm", (outtake1.getVelocity() * 60) / 28);
            telemetry.addData("outtake2 speed rpm", (outtake2.getVelocity() * 60) / 28);
            telemetry.addData("outtake1 speed tps", outtake1.getVelocity());
            telemetry.addData("outtake2 speed tps", outtake2.getVelocity());
            telemetry.update();
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