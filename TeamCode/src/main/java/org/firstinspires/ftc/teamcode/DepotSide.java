package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.AlmondLinear.mineralPosition.MIDDLE;

@Autonomous(name="DepotSide ",group="test")
public class DepotSide extends AlmondLinear
{

    public void runOpMode() throws InterruptedException
    {

        hardwareMap();
        teamMarker.setPosition(1);
        waitForStart();
        while (opModeIsActive() && isRunning)
        {
            detectorEnable();
            /*
            lScrew.setPower(-1);
            sleep(11000);
            lScrew.setPower(0);
            */
            driveToPosition(700,700,700,700,1);
            driveToPosition(1000,-1000,-1000,1000, 1);
            driveToPosition(-1000,-1000,-1000,-1000,1);

            if(detector.isFound()&&detector.getWidth()>40)
            {
                detector.disable();
                driveToPosition(8000,-8000,-8000,8000,1);
                driveToPosition(-1350,-1350,1350,1350,1);
                teamMarker.setPosition(0);
                driveToPosition(700,700,700,700,1);
                driveToPosition(1000,-1000,-1000,1000,1);
                driveToPosition(5000,5000,5000,5000,1);

            } else {
                driveToPosition(1200,1200,-1200,-1200,1);
                if(detector.isFound()&&detector.getWidth()>40)
                {

                    driveToPosition(5000,-5000,-5000,5000,1);
                    driveToPosition(1000,-1000,-1000,-1000,1);
                    driveToPosition(1000,-1000,-1000,1000,1);
                    driveToPosition(5000,5000,5000,5000,1);
                    teamMarker.setPosition(0);
                    wait(500);
                    driveToPosition(-2700,-2700,2700,2700,1);
                    driveToPosition(5000,5000,5000,5000,1);
                } else {
                    detector.disable();
                    driveToPosition(-2400,-2400,2400,2400,1);
                    driveToPosition(5000,-5000,-5000,5000,1);
                    driveToPosition(2000,-2000,-2000,2000,1);
                    driveToPosition(-4000,-4000,-4000,-4000,1);
                    driveToPosition(10000,10000,10000,10000,1);

                }

            }




            detector.disable();
            isRunning = false;
            stop();

        }

    }
}
