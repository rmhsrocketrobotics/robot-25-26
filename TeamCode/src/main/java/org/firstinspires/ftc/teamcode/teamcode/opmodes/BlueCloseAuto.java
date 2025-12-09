package org.firstinspires.ftc.teamcode.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.teamcode.MainAuto;

@Autonomous(preselectTeleOp = "MainTeleopBlue", group = "!", name = "BLUE CLOSE Auto")
public class BlueCloseAuto extends MainAuto {
    @Override
    public boolean allianceIsRed() {
        return false;
    }

    @Override
    public boolean useFarAuto() {
        return false;
    }
}