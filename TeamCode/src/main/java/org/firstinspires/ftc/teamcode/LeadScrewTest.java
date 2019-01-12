package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(group="test",name="LeadScrewTest")
public class LeadScrewTest extends AlmondLinear {
    public void runOpMode() throws InterruptedException{
        hardwareMap();
        lScrew.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lScrew.setPower(-1);
        sleep(10000);
    }
}
