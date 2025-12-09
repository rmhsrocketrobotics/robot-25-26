package org.firstinspires.ftc.teamcode.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teamcode.MainTeleop;

@TeleOp(group = "!", name = "RED TeleOp")
public class RedTeleop extends MainTeleop {
    @Override
    public boolean allianceIsRed() {
        return true;
    }
}