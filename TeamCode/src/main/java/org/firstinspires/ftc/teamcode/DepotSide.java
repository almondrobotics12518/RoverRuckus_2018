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
            driveToPosition(500,500,500,500,1);
            driveToPosition(1000,-1000,-1000,1000, 1);
            driveToPosition(-1000,-1000,-1000,-1000,1);

            if(detector.isFound()&&detector.getWidth()>40)
            {
                detector.disable();
                //adjustment for the robot not strafing properly
                driveToPosition(-500,-500,-500,-500,1);

                driveToPosition(8000,-8000,-8000,8000,1);
                driveToPosition(500,500,500,500,1);
                teamMarker.setPosition(0);
                sleep(500);
                driveToPosition(-1350,-1350,1350,1350,1);
                driveToPosition(-500,-500,-500,-500,1);
                driveToPosition(1500,1500,1500,1500,1);
                driveToPosition(500,-500,-500,500,1);
                driveToPosition(7000,7000,7000,7000,1);


            } else {
                driveToPosition(1300,1300,-1300,-1300,1);
                if(detector.isFound()&&detector.getWidth()>40)
                {
                    driveToPosition(7000,-5000,-5000,5000,1);
                    driveToPosition(150,150,-150,-150,1);

                } else {
                    detector.disable();
                    driveToPosition(-7000,-2600,2600,2600,1);
                    driveToPosition(5000,-5000,-5000,5000,1);


                }

            }




            detector.disable();
            isRunning = false;
            stop();

        }

    }
}
