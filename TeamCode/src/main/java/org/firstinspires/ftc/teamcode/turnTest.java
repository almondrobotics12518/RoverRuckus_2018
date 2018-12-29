package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(group = "test",name="turnTest")
public class turnTest extends AlmondLinear {

    public void runOpMode() throws InterruptedException
    {
        hardwareMap();
        initImu();
        turnToAngleAbsolute(90,1);
        turnToAngleRelative(90,0.9);
        turnToAngleAbsolute(90,1);
    }
}
