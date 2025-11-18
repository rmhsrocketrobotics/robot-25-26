package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp(group = "!main")

public class BasicTeleop extends LinearOpMode{
    @Override
    public void runOpMode() {
        DcMotor flDrivetrain = hardwareMap.get(DcMotor.class, "fl");
        DcMotor frDrivetrain = hardwareMap.get(DcMotor.class, "fr");
        DcMotor blDrivetrain = hardwareMap.get(DcMotor.class, "bl");
        DcMotor brDrivetrain = hardwareMap.get(DcMotor.class, "br");

        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        DcMotor outtake = hardwareMap.get(DcMotor.class, "outtake");
        DcMotor flick = hardwareMap.get(DcMotor.class, "flick");

        //random ahh imu stuff dw
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));

        imu.initialize(parameters);
        imu.resetYaw();

        //change the direction of drivetrain motors so they're all facing the same way
        flDrivetrain.setDirection(DcMotor.Direction.REVERSE);
        blDrivetrain.setDirection(DcMotor.Direction.REVERSE);

        //make the motors brake
        flDrivetrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frDrivetrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blDrivetrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brDrivetrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flDrivetrain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frDrivetrain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blDrivetrain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brDrivetrain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        flDrivetrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frDrivetrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        blDrivetrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brDrivetrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            double yPower = -gamepad1.right_stick_y;
            double xPower = gamepad1.right_stick_x;
            double rPower = gamepad1.left_stick_x;

            //equations to move the robot
            flDrivetrain.setPower(yPower + xPower + rPower);
            frDrivetrain.setPower(yPower - xPower - rPower);
            blDrivetrain.setPower(yPower - xPower + rPower);
            brDrivetrain.setPower(yPower + xPower - rPower);

            flick.setPower(-gamepad2.left_stick_y);
            intake.setPower(-gamepad2.right_stick_y);

            telemetry.addData("fldrivetrain is ", flDrivetrain.getCurrentPosition());
            telemetry.addData("frdrivetrain is ", frDrivetrain.getCurrentPosition());
            telemetry.addData("bldrivetrain is ", blDrivetrain.getCurrentPosition());
            telemetry.addData("brdrivetrain is ", brDrivetrain.getCurrentPosition());
            telemetry.update();
        }
    }
}