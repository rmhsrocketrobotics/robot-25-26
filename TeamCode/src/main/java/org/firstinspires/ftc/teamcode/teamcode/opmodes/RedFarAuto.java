package org.firstinspires.ftc.teamcode.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.teamcode.MainAuto;

@Autonomous(preselectTeleOp = "RED TeleOp", group = "!", name = "RED FAR Auto")
public class RedFarAuto extends MainAuto {
    @Override
    public boolean allianceIsRed() {
        return true;
    }

    @Override
    public boolean useFarAuto() {
        return true;
    }
}