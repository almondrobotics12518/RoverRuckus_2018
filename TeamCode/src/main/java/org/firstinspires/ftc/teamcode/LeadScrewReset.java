package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
@Autonomous
public class LeadScrewReset extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor lScrew;
        lScrew = hardwareMap.dcMotor.get("LScrew");
        waitForStart();
        lScrew.setPower(1);
        sleep(11500);
    }
}