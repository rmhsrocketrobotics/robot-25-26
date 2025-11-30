package org.firstinspires.ftc.teamcode.teamcode.subsystems;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayDeque;
import java.util.Deque;

public class Spindex {
    private ServoImplEx drumServo;

    final double[] intakePositions = {0, 0.3826, 0.7831};
    final double[] outtakePositions = {0.5745, 0.975, 0.1823};

    public DcMotor intake;

    DcMotor flick;

    NormalizedColorSensor intakeColorSensor;

    NormalizedColorSensor outtakeColorSensor;

    private String drumMode;

    int drumPosition;

    String[] ballStates = {"empty", "empty", "empty"};
    public Deque<String> ballQueue = new ArrayDeque<>();

    private ElapsedTime switchCooldownTimer;

    // time it takes to go from position 0 to position 1 on the drum servo
    // (setting too low will mean the sensor sees the same ball multiple times)
    final double switchCooldownConstant = 1.3;

    double switchCooldown = switchCooldownConstant;

    private ElapsedTime flickTimer;

    final double flickTime = 1.2; // time that the robot will try to flick the ball up for
    final double postFlickTime = 0.25; // time that the robot will wait after flicking before doing anything else

    public boolean shouldSwitchToIntake = false;
    public boolean shouldSwitchToOuttake = false;

    /**
     * should run BEFORE waitForStart()
     * */
    public Spindex(HardwareMap hardwareMap) {
        drumServo = hardwareMap.get(ServoImplEx.class, "drumServo");
        drumServo.setPwmRange(new PwmControl.PwmRange(500, 2500));

        intake = hardwareMap.get(DcMotor.class, "intake");
        //intake.setDirection(DcMotor.Direction.REVERSE);

        flick = hardwareMap.get(DcMotor.class, "flick");
        flick.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeColorSensor = hardwareMap.get(NormalizedColorSensor.class, "intakeColorSensor");
        intakeColorSensor.setGain(15);

        //outtakeColorSensor = hardwareMap.get(NormalizedColorSensor.class, "idk lmao");

        drumMode = "intake";
        drumPosition = 0;
    }

    /**
     * should run AFTER waitForStart()
     * */
    public void init() {
        switchCooldownTimer = new ElapsedTime();
        flickTimer = new ElapsedTime(676767);
        updateDrumPosition();
    }

    /**
     * updates drum position based on drumMode and drumPosition
     * <p>
     * gets ran once at the end of update()
     * */
    private void updateDrumPosition() {
        if (drumInIntakeMode()) {
            drumServo.setPosition(intakePositions[drumPosition]);
        }
        else if (drumInOuttakeMode()) {
            drumServo.setPosition(outtakePositions[drumPosition]);
        }
    }

    /**
     * returns the physical position of the drum at a certain mode/position
     * */
    private double getDrumPositions(String drumMode, int drumPosition) {
        if (drumMode.equals("intake")) {
            return intakePositions[drumPosition];
        } else if (drumMode.equals("outtake")) {
            return outtakePositions[drumPosition];
        }
        return 676767;
    }

    public void incrementDrumPosition() {
        int newDrumPosition = drumPosition + 1;
        String newDrumState = drumMode;
        if (newDrumPosition >= 3) {
            newDrumPosition = 0;

            if (drumInIntakeMode()) {
                newDrumState = "outtake";
            } else if (drumInOuttakeMode()) {
                newDrumState = "intake";
            }
        }

        setDrumState(newDrumState, newDrumPosition);
    }

    public boolean drumInOuttakeMode() {
        return drumMode.equals("outtake");
    }

    public boolean drumInIntakeMode() {
        return drumMode.equals("intake");
    }

    public boolean drumIsEmpty() {
        return ballStates[0].equals("empty") && ballStates[1].equals("empty") && ballStates[2].equals("empty");
    }

    public void setDrumState(String newDrumMode) {
        if (!this.drumMode.equals(newDrumMode)) {
            switchCooldown = switchCooldownConstant * (Math.abs(getDrumPositions(newDrumMode, this.drumPosition) - getDrumPositions(this.drumMode, this.drumPosition)));
            switchCooldownTimer.reset();

            this.drumMode = newDrumMode;
        }

    }

    public void setDrumState(int newDrumPosition) {
        if (this.drumPosition != newDrumPosition) {
            switchCooldown = switchCooldownConstant * (Math.abs(getDrumPositions(this.drumMode, newDrumPosition) - getDrumPositions(this.drumMode, this.drumPosition)));
            switchCooldownTimer.reset();

            this.drumPosition = newDrumPosition;
        }

    }

    public void setDrumState(String newDrumMode, int newDrumPosition) {
        if ((!this.drumMode.equals(newDrumMode)) || (this.drumPosition != newDrumPosition)) {
            switchCooldown = switchCooldownConstant * (Math.abs(getDrumPositions(newDrumMode, newDrumPosition) - getDrumPositions(this.drumMode, this.drumPosition)));
            switchCooldownTimer.reset();

            this.drumMode = newDrumMode;
            this.drumPosition = newDrumPosition;
        }
    }

    /**
     * shoot a ball
     * */
    private void setDrumStateToNextOuttake() {
        for (int i = 0; i < ballStates.length; i++) {
            String ballState = ballStates[i];
            if (!ballState.equals("empty")) {
                setDrumState("outtake", i);
                return;
            }
        }
    }

    /**
     * shoot a ball of a specific color
     * if there are no balls of the specified color and shootAny is true, then the robot will attempt to shoot a ball of the opposite color
     * */
    private void setDrumStateToNextOuttake(String color, boolean shootAny) {
        for (int i = 0; i < ballStates.length; i++) {
            String ballState = ballStates[i];
            if (ballState.equals(color)) {
                setDrumState("outtake", i);
                return;
            }
        }

        if (shootAny) {
            setDrumStateToNextOuttake();
        }
    }

    public boolean drumIsSwitching() {
        return switchCooldownTimer.seconds() < switchCooldown;
    }

    public boolean drumIsFlicking() {
        return flickTimer.seconds() < flickTime;
    }

    public boolean drumIsInFlickCooldown() {
        return flickTimer.seconds() < flickTime + postFlickTime;
    }

    /**
     * returns true if a ball is detected
     * */
    private boolean detectBallIntake() {
        // if still in cooldown, return
        if (drumIsSwitching()) {
            return false;
        }

        // get distance from sensor
        boolean ballDetected = ((DistanceSensor) intakeColorSensor).getDistance(DistanceUnit.CM) < 3;

        if (!ballDetected) { // could add && Objects.equals(ballStates[drumPosition], "empty") to this statement
            return false;
        }

        // get color from sensor
        NormalizedRGBA colors = intakeColorSensor.getNormalizedColors();
        final float[] hsvValues = new float[3];
        Color.colorToHSV(colors.toColor(), hsvValues);

        // assume a ball is purple unless proven green (most balls on the field are purple)
        String color = "purple";
        if (hsvValues[0] < 180) {
            color = "green";
        }

        ballStates[drumPosition] = color;

        return true;
    }

//    /**
//     * move the drum to the next outtake slot with a ball in it and flick it
//     * */
//    public void flickNextBall() {
//        setDrumStateToNextOuttake();
//        flickTimer.reset();
//    }

    /**
     * move the drum to the next outtake slot with a ball of a certain color in it and flick it
     * if there are no balls of the specified color and shootAny is true, then the robot will attempt to shoot a ball of the opposite color
     */
    public void flickNextBall(String color, boolean shootAny) {
        if (!drumIsEmpty()) {
        setDrumStateToNextOuttake(color, shootAny);
        flickTimer.reset();
        }
    }

    public void queueBall(String color) {
        if (color.equals("green")) {
            ballQueue.addLast("green");
        } else if (color.equals("purple")) {
            ballQueue.addLast("purple");
        }
    }

    /**
     * should be called in the event loop
     * */
    public void update(Outtake outtake) {
        shouldSwitchToIntake = false;
        shouldSwitchToOuttake = false;

        // detect if we have intaked (intook?) a ball, if so, switch to a new slot
        if (drumInIntakeMode()) {
            if (detectBallIntake()) {
                incrementDrumPosition();
                if (drumInOuttakeMode()) {
                    shouldSwitchToOuttake = true;
                }
            }
        }

        // if we aren't doing anything, get the next ball from the queue
        // holy if statement
        if (drumInOuttakeMode() && !drumIsSwitching() && !drumIsInFlickCooldown() && !ballQueue.isEmpty() && outtake.atTargetSpeed()) {
            flickNextBall(ballQueue.removeFirst(), true);
        }

        // if we should be flicking, then turn the flick on
        if (drumInOuttakeMode() && drumIsFlicking()) {
            if (!drumIsSwitching()) {
                flick.setPower(1);
                ballStates[drumPosition] = "empty";
            } else {
                flickTimer.reset();
            }
        } else {
            flick.setPower(0);
        }

        // switch back to intake if we're not doing anything
        if (drumIsEmpty() && drumInOuttakeMode() && !drumIsSwitching() && !drumIsInFlickCooldown()) {
            shouldSwitchToIntake = true;
        }

        updateDrumPosition();
    }

    public void printTelemetry(Telemetry telemetry) {
        telemetry.addData("ballStates[0]", ballStates[0]);
        telemetry.addData("ballStates[1]", ballStates[1]);
        telemetry.addData("ballStates[2]", ballStates[2]);
        telemetry.addData("drumMode", drumMode);
        telemetry.addData("drumPosition", drumPosition);
        telemetry.addData("switchCooldown", switchCooldown);
        telemetry.addData("switchCooldownTimer.seconds()", switchCooldownTimer.seconds());
    }
}
