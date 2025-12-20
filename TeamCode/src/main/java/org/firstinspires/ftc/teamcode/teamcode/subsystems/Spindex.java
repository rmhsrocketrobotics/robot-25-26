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

    public enum BallState {
        GREEN,
        PURPLE,
        EMPTY
    }

    private enum SpindexState {
        INTAKE,
        SPIN_UP,
        OUTTAKE
    }

    private SpindexState spindexState = SpindexState.INTAKE;

    private ServoImplEx drumServo;

    final double[] intakePositions = {0, 0.3826, 0.7831};
    final double[] outtakePositions = {0.5745, 0.975, 0.1823};

    public DcMotor intake;

    public DcMotor flick;

    NormalizedColorSensor intakeColorSensor;

    private String drumMode;

    private int drumPosition;

    public BallState[] ballStates = {BallState.EMPTY, BallState.EMPTY, BallState.EMPTY};
    public Deque<BallState> ballQueue = new ArrayDeque<>();

    private ElapsedTime switchCooldownTimer;

    // time it takes to go from position 0.0 to position 1.0 on the drum servo
    // (setting too low will mean the sensor sees the same ball multiple times)
    public double switchCooldownConstant = 1.4;//1.75;

    double switchCooldown = switchCooldownConstant;

    public ElapsedTime outtakeTimer;
    public double outtakeTime = 0.6;

    public ElapsedTime spinUpTimer;
    public double spinUpTime = 0.6;

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
        drumPosition = 2;
    }

    /**
     * should run AFTER waitForStart()
     * */
    public void init() {
        switchCooldownTimer = new ElapsedTime();
        outtakeTimer = new ElapsedTime(676767);
        spinUpTimer = new ElapsedTime(676767);
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

    public void nextDrumPosition() {
        if (drumInIntakeMode()) {
            int newDrumPosition = drumPosition - 1;
            if (newDrumPosition < 0) {
                newDrumPosition = 0;
            }
            setDrumState(newDrumPosition);

        } else if (drumInOuttakeMode()) {
            // this corresponds to the way that the drum looks irl; the drum outtakes the ball in slot 2, then in slot 0, then in slot 1
            if (drumPosition == 2) {
                setDrumState(0);
            } else if (drumPosition == 0) {
                setDrumState(1);
            } else if (drumPosition == 1) {
                //setDrumState(1);
            }
        }

    }

    public boolean drumInOuttakeMode() {
        return drumMode.equals("outtake");
    }

    public boolean drumInIntakeMode() {
        return drumMode.equals("intake");
    }

    public boolean drumIsEmpty() {
        return ballStates[0] == BallState.EMPTY && ballStates[1] == BallState.EMPTY && ballStates[2] == BallState.EMPTY;
    }

    public boolean drumIsFull() {
        return ballStates[0] != BallState.EMPTY && ballStates[1] != BallState.EMPTY && ballStates[2] != BallState.EMPTY;
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

    public void setDrumStateToSpinUpPosition() {
        if (ballStates[0] == ballStates[1] && ballStates[1] == ballStates[2]) {
            setDrumState("intake", 0);
        } else if (ballStates[0] == ballStates[1]) {
            setDrumState("intake", 1);
        } else if (ballStates[1] == ballStates[2]) {
            setDrumState("intake", 2);
        } else {
            setDrumState("intake", 1);
        }
    }

    /**
     * shoot a ball
     * */
//    private void setDrumStateToNextOuttake() {
//        for (int i = 0; i < ballStates.length; i++) {
//            BallState ballState = ballStates[i];
//            if (ballState != BallState.EMPTY) {
//                setDrumState("outtake", i);
//                return;
//            }
//        }
//    }

    /**
     * shoot a ball of a specific color
     * if there are no balls of the specified color and shootAny is true, then the robot will attempt to shoot a ball of the opposite color
     * */
//    private void setDrumStateToNextOuttake(BallState color, boolean shootAny) {
//        for (int i = 0; i < ballStates.length; i++) {
//            BallState ballState = ballStates[i];
//            if (ballState == color) {
//                setDrumState("outtake", i);
//                return;
//            }
//        }
//
//        if (shootAny) {
//            setDrumStateToNextOuttake();
//        }
//    }

    private void setDrumStateToOuttake(BallState color) {
        /// the drum positions in real life go:
        /// in 0 | out 2 | in 1 | out 0 | in 2 | out 1

        if (drumMode.equals("intake")) {
            if ((drumPosition == 1 || drumPosition == 2) && ballStates[0] == color) {
                setDrumState("outtake", 0);
            }


        } else if (drumMode.equals("outtake")) {

        }
    }

    public boolean drumIsSwitching() {
        return switchCooldownTimer.seconds() < switchCooldown;
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
        BallState color = BallState.PURPLE;
        if (hsvValues[0] < 180) {
            color = BallState.GREEN;
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
//    public void flickNextBall(BallState color, boolean shootAny) {
//        if (!drumIsEmpty()) {
//        setDrumStateToNextOuttake(color, shootAny);
//        flickTimer.reset();
//        }
//    }

    public void queueBall(BallState color) {
        if (color == BallState.GREEN) {
            ballQueue.addLast(BallState.GREEN);
        } else if (color == BallState.PURPLE) {
            ballQueue.addLast(BallState.PURPLE);
        }
    }

    public void queueBall(String color) {
        if (color.equals("green")) {
            ballQueue.addLast(BallState.GREEN);
        } else if (color.equals("purple")) {
            ballQueue.addLast(BallState.PURPLE);
        }
    }

    public void forceSwitchToStateIntake() {
        spindexState = SpindexState.INTAKE;
        setDrumState("intake", 2);

        ballStates[0] = BallState.EMPTY;
        ballStates[1] = BallState.EMPTY;
        ballStates[2] = BallState.EMPTY;

        flick.setPower(0);
    }

    public void forceSwitchToStateOuttake() {
        spindexState = SpindexState.SPIN_UP;
        setDrumState("intake", 0);
        ballQueue.clear();
        flick.setPower(1);
    }

    /**
     * should be called in the event loop
     * */
    public void update(Outtake outtake) {
        shouldSwitchToIntake = false;
        shouldSwitchToOuttake = false;

        switch(spindexState) {
            case INTAKE:
                boolean ballDetected = detectBallIntake();
                if (ballDetected && !drumIsSwitching()) { // auto move drum when ball detected
                    nextDrumPosition();
                }

                if (drumIsFull() && !drumIsSwitching()) { // auto switch to spinning up state when drum is full
                    shouldSwitchToOuttake = true;
                    spindexState = SpindexState.SPIN_UP;
                    setDrumState("intake", 0);
                    ballQueue.clear();
                    spinUpTimer.reset();
                }

                flick.setPower(0);
                break;

            case SPIN_UP:
                if (spinUpTimer.seconds() > spinUpTime && outtake.atTargetSpeed() && !ballQueue.isEmpty() && !drumIsSwitching()) { // auto switch to outtake state when ready to launch
                    spindexState = SpindexState.OUTTAKE;

                    ballQueue.removeFirst();

                    if (drumInIntakeMode()) {
                        setDrumState("outtake", 2);
                    } else {
                        nextDrumPosition();
                    }

                    ballStates[drumPosition] = BallState.EMPTY;

                    outtakeTimer.reset();
                }

                if (!drumIsSwitching()) {
                    flick.setPower(1);
                }

                break;

            case OUTTAKE:
                if (outtakeTimer.seconds() > outtakeTime && !drumIsSwitching()) { // auto switch back to intake or spin up state
                    if (drumIsEmpty()) {
                        spindexState = SpindexState.INTAKE;
                        shouldSwitchToIntake = true;
                        setDrumState("intake", 2);

                    } else {
                        spindexState = SpindexState.SPIN_UP;
                    }
                }

                flick.setPower(1);
                break;
        }

        updateDrumPosition();
    }

    public void printTelemetry(Telemetry telemetry) {
        for (BallState ballState : ballStates) {
            if (ballState == BallState.GREEN) {
                telemetry.addData("ball", "green");
            } else if (ballState == BallState.PURPLE) {
                telemetry.addData("ball", "purple");
            } else {
                telemetry.addData("ball", "empty");
            }
        }
        telemetry.addData("drumMode", drumMode);
        telemetry.addData("drumPosition", drumPosition);
        telemetry.addData("switchCooldown", switchCooldown);
        telemetry.addData("switchCooldownTimer.seconds()", switchCooldownTimer.seconds());
    }
}
