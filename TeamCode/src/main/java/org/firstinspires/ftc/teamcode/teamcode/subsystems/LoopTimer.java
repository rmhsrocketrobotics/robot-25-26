package org.firstinspires.ftc.teamcode.teamcode.subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

public class LoopTimer {
    private final ElapsedTime timer;

    public LoopTimer() {
        timer = new ElapsedTime();
    }

    /// returns the time since this method was last called
    public double getLoopTimeSeconds() {
        double loopTime = timer.seconds();
        timer.reset();
        return loopTime;
    }
}
