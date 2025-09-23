package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp(group = "!main")

public class BasicTeleop extends LinearOpMode{
    @Override
    public void runOpMode() {
        //get the motors/servos from the driver station so we can control them
        // DcMotor armMotor = hardwareMap.get(DcMotor.class, "arm");
        // DcMotor slideMotor = hardwareMap.get(DcMotor.class, "slide");

        DcMotor flDrivetrain = hardwareMap.get(DcMotor.class, "fl");
        DcMotor frDrivetrain = hardwareMap.get(DcMotor.class, "fr");
        DcMotor blDrivetrain = hardwareMap.get(DcMotor.class, "bl");
        DcMotor brDrivetrain = hardwareMap.get(DcMotor.class, "br");

        // Servo leftClaw = hardwareMap.get(Servo.class, "lc");
        // Servo rightClaw = hardwareMap.get(Servo.class, "rc");

        // CRServo outakeServo = hardwareMap.get(CRServo.class, "outakeServo");

        //random ahh imu stuff dw
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));

        imu.initialize(parameters);
        imu.resetYaw();

        //gamepad1.left_bumper;
        //gamepad1.left_trigger;

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

        // flDrivetrain.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // frDrivetrain.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // blDrivetrain.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // brDrivetrain.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        flDrivetrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frDrivetrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        blDrivetrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brDrivetrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();

        // leftClaw.setPosition(0.5); //open position is 0.243; closed position is 0.426
        // rightClaw.setPosition(0.5); //open position is 0.4; closed position is 0.233

        boolean aPressedLast = false;

        while (opModeIsActive()) {
            double yPower = CustomMath.funnyPiecewiseFunction(-gamepad1.right_stick_y) * 0.5;
            double xPower = CustomMath.funnyPiecewiseFunction(gamepad1.right_stick_x) * 0.5;
            double rPower = CustomMath.funnyPiecewiseFunction(gamepad1.left_stick_x) * 0.5;

            if (!aPressedLast && gamepad1.a) {
                //do stuff!
            }

            //equations to move the robot
            flDrivetrain.setPower(yPower + xPower + rPower);
            frDrivetrain.setPower(yPower - xPower - rPower);
            blDrivetrain.setPower(yPower - xPower + rPower);
            brDrivetrain.setPower(yPower + xPower - rPower);

            flDrivetrain.setTargetPosition(300);

            telemetry.addData("fldrivetrain is ", flDrivetrain.getCurrentPosition());
            telemetry.addData("frdrivetrain is ", frDrivetrain.getCurrentPosition());
            telemetry.addData("bldrivetrain is ", blDrivetrain.getCurrentPosition());
            telemetry.addData("brdrivetrain is ", brDrivetrain.getCurrentPosition());
            telemetry.update();

            //keep track of what buttons were pressed last loop
            aPressedLast = gamepad1.a;
        }
    }
}

class CustomMath {
    public static double funnyPiecewiseFunction(double power){
        double deadzone = 0.075;
        double startPower = 0.3;

        if (power > 1) {power = 1;}
        else if (power < -1) {power = -1;}

        if (power < -deadzone){
            return lineBetweenPoints(-deadzone, -startPower, -1, -1, power);

        } else if (power > deadzone) {
            return lineBetweenPoints(deadzone, startPower, 1, 1, power);

        } else {
            return 0;
        }
    }

    public static double lineBetweenPoints(double x1, double y1, double x2, double y2, double x){
        double slope = (y1-y2)/(x1-x2);
        double yIntercept = y1 - (slope * x1);

        return (slope * x) + yIntercept;
    }
}