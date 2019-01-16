package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "SideWaysTest",group = "test")
public class SideWaysTest extends AlmondLinear {

    public void runOpMode() throws InterruptedException
    {
        int leftFrontEncoder;
        hardwareMap();
        setModeRunUsingEncoders();
        leftFront.setPower(1);
        while(leftFront.getCurrentPosition()<2000){
            sleep(20);
        }
        leftFrontEncoder = leftFront.getCurrentPosition()+2000;
        leftFront.setPower(0);
        leftFront.setPower(1);
        while(leftFront.getCurrentPosition()<leftFrontEncoder){
            sleep(20);
        }
        leftFront.setPower(0);
    }
}
