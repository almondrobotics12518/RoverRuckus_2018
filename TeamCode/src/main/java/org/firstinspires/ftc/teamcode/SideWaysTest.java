package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "SideWaysTest",group = "test")
public class SideWaysTest extends AlmondLinear {

    public void runOpMode() throws InterruptedException
    {
        hardwareMap();
        waitForStart();
        driveToPosition(5000,-5000,-5000,5000,1);
    }
}
