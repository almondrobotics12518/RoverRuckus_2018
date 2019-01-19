package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
@Autonomous(name="ResetLeadScrew",group="test")
public class LeadScrewReset extends AlmondLinear {

    @Override
    public void runOpMode() throws InterruptedException {
        hardwareMap();
        waitForStart();
        lScrew.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lScrew.setPower(-1);
        sleep(5500);

    }
}