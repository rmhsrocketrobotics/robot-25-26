package org.firstinspires.ftc.teamcode.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teamcode.MainTeleop;

@TeleOp(group = "!", name = "BLUE TeleOp")
public class BlueTeleop extends MainTeleop {
    @Override
    public boolean allianceIsRed() {
        return false;
    }
}