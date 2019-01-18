package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "SideWaysTest",group = "test")
public class SideWaysTest extends AlmondLinear {

    public void runOpMode() throws InterruptedException
    {

        hardwareMap();
        setModeRunUsingEncoders();
        waitForStart();
        while(opModeIsActive()&&isRunning){
            encoderDrive(10000,-10000,-10000,10000,1);
            encoderDrive(-10000,10000,10000,-10000,1);
            isRunning=false;
        }
        setPowerAll(0);
    }
}
