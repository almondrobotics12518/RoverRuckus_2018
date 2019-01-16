package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(group="opmodes",name="Back-up")
public class LeadScrewTest extends AlmondLinear {
    public void runOpMode() throws InterruptedException{
        while(isRunning&&opModeIsActive()){
            hardwareMap();
            lScrew.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lScrew.setPower(-1);
            sleep(5000);
        }

    }
}
