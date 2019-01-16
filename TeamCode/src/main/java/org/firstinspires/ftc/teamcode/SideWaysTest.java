package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "SideWaysTest",group = "test")
public class SideWaysTest extends AlmondLinear {

    public void runOpMode() throws InterruptedException
    {

        hardwareMap();
        setModeRunUsingEncoders();
        waitForStart();
        encoderDrive(1000,1000,1000,1000,1);
        requestOpModeStop();
    }
}
