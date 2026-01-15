package org.firstinspires.ftc.teamcode.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.teamcode.MainAuto;

@Autonomous(preselectTeleOp = "RED TeleOp", group = "!", name = "RED CLOSE Auto")
public class RedCloseAuto extends MainAuto {
    @Override
    public boolean allianceIsRed() {
        return true;
    }

    @Override
    public boolean useFarAuto() {
        return false;
    }
}