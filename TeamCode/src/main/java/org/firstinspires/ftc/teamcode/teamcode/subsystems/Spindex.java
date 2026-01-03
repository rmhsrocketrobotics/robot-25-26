package org.firstinspires.ftc.teamcode.teamcode.subsystems;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.teamcode.PoseStorage;

import java.util.ArrayDeque;
import java.util.Deque;

public class Spindex {

    public enum BallState {
        GREEN,
        PURPLE,
        EMPTY
    }

    private ServoImplEx drumServo;

    final double[] intakePositions = {0, 0.3826, 0.7831};
    // final double[] outtakePositions = {0.5745, 0.975, 0.1823}; old values
    final double[] outtakePositions = {0.5845, 0.98, 0.15};

    public DcMotor intakeMotor;

    public Servo flickServo;

    NormalizedColorSensor intakeColorSensor;

    private String drumMode;

    private int drumPosition;

    public BallState[] ballStates = PoseStorage.ballStates.clone();
    public Deque<BallState> ballQueue = new ArrayDeque<>();

    private ElapsedTime switchCooldownTimer;

    // time it takes to go from position 0.0 to position 1.0 on the drum servo
    // (setting too low will mean the sensor sees the same ball multiple times)
    public double switchCooldownConstant = 1.4;//1.75;

    double switchCooldown = switchCooldownConstant;

    public ElapsedTime flickTimer;
    // time that the flick has to go up and down
    public double flickTime = 0.7;

    public boolean shouldSwitchToIntake = false;
    public boolean shouldSwitchToOuttake = false;

    private State lastState = State.INTAKE;

    private boolean shootAll;
    private boolean waitingToFlick = false;

    /**
     * should run BEFORE waitForStart()
     * */
    public Spindex(HardwareMap hardwareMap) {
        PoseStorage.ballStates = new BallState[]{BallState.EMPTY, BallState.EMPTY, BallState.EMPTY};

        drumServo = hardwareMap.get(ServoImplEx.class, "drumServo");
        drumServo.setPwmRange(new PwmControl.PwmRange(500, 2500));

        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        //intake.setDirection(DcMotor.Direction.REVERSE);

        flickServo = hardwareMap.get(Servo.class, "flick");

        intakeColorSensor = hardwareMap.get(NormalizedColorSensor.class, "intakeColorSensor");
        intakeColorSensor.setGain(15);

        //outtakeColorSensor = hardwareMap.get(NormalizedColorSensor.class, "idk lmao");

        drumMode = "intake";
        drumPosition = 2;

        shootAll = false;
    }

    /**
     * should run AFTER waitForStart()
     * */
    public void init() {
        switchCooldownTimer = new ElapsedTime();
        flickTimer = new ElapsedTime(676767);
        setFlickPosition("down");
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

    public boolean drumIsSwitching() {
        return switchCooldownTimer.seconds() < switchCooldown;
    }

    public boolean drumIsFlicking() {
        return flickTimer.seconds() < flickTime;
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

    private void setFlickPosition(String position) {
        if (position.equals("up")) {
            flickServo.setPosition(0.25);
        } else if (position.equals("down")) {
            flickServo.setPosition(0.58);
        }
    }

    /**
     * shoot a ball
     * */
    private void setDrumStateToNextOuttake() {
        for (int i = 0; i < ballStates.length; i++) {
            BallState ballState = ballStates[i];
            if (ballState != BallState.EMPTY) {
                setDrumState("outtake", i);
                return;
            }
        }
    }

    /**
     * shoot a ball of a specific color
     * if there are no balls of the specified color and shootAny is true, then the robot will attempt to shoot a ball of the opposite color
     * */
    private void setDrumStateToNextOuttake(BallState color, boolean shootAny) {
        for (int i = 0; i < ballStates.length; i++) {
            BallState ballState = ballStates[i];
            if (ballState == color) {
                setDrumState("outtake", i);
                return;
            }
        }

        if (shootAny) {
            setDrumStateToNextOuttake();
        }
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
        boolean ballDetected = ((DistanceSensor) intakeColorSensor).getDistance(DistanceUnit.CM) < 4; // og value: 3 cm

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

    public void shootAllBalls() {
        shootAll = true;
    }

    public void recheckDrum() {
        setDrumState("intake", 2);

        ballStates[0] = BallState.EMPTY;
        ballStates[1] = BallState.EMPTY;
        ballStates[2] = BallState.EMPTY;
    }

    /**
     * should be called in the event loop
     * */
    public void update(Outtake outtake, State state) {
        shouldSwitchToIntake = false;
        shouldSwitchToOuttake = false;

        switch (state) {
            case INTAKE:
                // when switching to INTAKE mode, reset the drum's position and reset the recorded ball states
                if (lastState == State.OUTTAKE) {
                    setDrumState("intake", 2);

                    ballStates[0] = BallState.EMPTY;
                    ballStates[1] = BallState.EMPTY;
                    ballStates[2] = BallState.EMPTY;
                }

                // auto move drum when ball detected
                if (!drumIsSwitching()) {
                    boolean ballDetected = detectBallIntake();
                    if (ballDetected) {
                        nextDrumPosition();
                    }
                }

                // suggest to switch to OUTTAKE mode when the drum is full
                if (!drumIsSwitching() && drumIsFull()) {
                    shouldSwitchToOuttake = true;
                }

                // ensure the queue stays clear when in INTAKE mode
                ballQueue.clear();

                // ensure the flick servo is down when in INTAKE mode
                setFlickPosition("down");

                // ensure shootAll is false when in INTAKE mode
                shootAll = false;

                break;

            case OUTTAKE:
                // when switching to OUTTAKE mode, reset the drum's position
                if (lastState == State.INTAKE) {
                    setDrumState("outtake", 2);
                    waitingToFlick = false;
                }

                // flick ball when there is a ball in the queue and the outtake is at the target speed
                if (!drumIsSwitching() && !waitingToFlick && !drumIsFlicking() && !drumIsEmpty()) {
                    if (shootAll) {
                        if (ballStates[drumPosition] == BallState.EMPTY) {
                            nextDrumPosition();
                        }

                        ballStates[drumPosition] = BallState.EMPTY;
                        waitingToFlick = true;

                    } else if (!ballQueue.isEmpty()) {
                        setDrumStateToNextOuttake(ballQueue.removeFirst(), true);

                        ballStates[drumPosition] = BallState.EMPTY;
                        waitingToFlick = true;
                    }
                }

                // if we should be flicking, but the outtake isn't at the right speed, or the drum is still switching,
                // then we wait and keep the flick down
                if (waitingToFlick && !drumIsSwitching() && outtake.atTargetSpeed()) {
                    flickTimer.reset();
                    waitingToFlick = false;
                }

                // set the flick's position based on the flick timer
                if (flickTimer.seconds() < (flickTime / 2)) {
                    setFlickPosition("up");
                } else {
                    setFlickPosition("down");
                }

                // suggest to swtich back to INTAKE when the drum is empty
                if (!drumIsSwitching() && !waitingToFlick && !drumIsFlicking() && drumIsEmpty()) {
                    shouldSwitchToIntake = true;
                }

                break;
        }

        updateDrumPosition();

        lastState = state;
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
