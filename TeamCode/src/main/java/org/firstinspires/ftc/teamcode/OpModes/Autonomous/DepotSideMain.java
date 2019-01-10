package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AlmondLinear;
import org.firstinspires.ftc.teamcode.DataLogThread2;

import static org.firstinspires.ftc.teamcode.AlmondLinear.mineralPosition.MIDDLE;

@Autonomous(name="DepotSide ",group="opmodes")
public class DepotSideMain extends AlmondLinear
{

    DataLogThread2 log;
    public void runOpMode() throws InterruptedException
    {

        hardwareMap();
        log = new DataLogThread2("DepotSideAuto",250,leftFront,leftBack,rightFront,rightBack,lScrew);
        log.start();
        teamMarker.setPosition(0.6);
        waitForStart();
        while (opModeIsActive() && isRunning)
        {
            telemetry.addData("Status","In Start");
            telemetry.update();
            detectorEnable();



            /*
            lScrew.setPower(-1);
            sleep(11000);
            lScrew.setPower(0);
            */
            driveToPosition(500,500,500,500,1);
            driveToPosition(1000,-1000,-1000,1000, 1);
            driveToPosition(-1000,-1000,-1000,-1000,1);

            if(detector.isFound()&&detector.getWidth()>40&&detector.getXPosition()<500&&detector.getXPosition()>100)
            {
                detector.disable();
                //adjustment for the robot not strafing properly
                driveToPosition(-500,-500,-500,-500,1);

                driveToPosition(9000,-9000,-9000,9000,1);
                driveToPosition(-1000,1000,1000,-1000,1);
                driveToPosition(1000,1000,1000,1000,1);
                teamMarker.setPosition(0);
                sleep(500);
                driveToPosition(2000,2000,2000,2000,1);
                driveToPosition(-1350,-1350,1350,1350,1);
                driveToPosition(5000,5000,5000,5000,1);

                log.setIsRunning(false);
                log.closeLogFile();

            } else {
                driveToPosition(-1300,-1300,1300,1300,0.5);
                driveToPosition(-1200,-1200,-1200,-1200,1);
                if(detector.isFound()&&detector.getWidth()>40)
                {
                    detector.disable();

                    driveToPosition(7500,-7500,-7500,7500,1);
                    driveToPosition(-1000,1000,1000,-1000,1);
                    driveToPosition(-3000,-3000,-3000,-3000,1);
                    teamMarker.setPosition(0);
                    sleep(400);
                    driveToPosition(6000,6000,6000,6000,1);
                    driveToPosition(1300,-1300,-1300,1300,1);
                    driveToPosition(3000,3000,3000,3000,1);

                    log.setIsRunning(false);
                    log.closeLogFile();

                } else {
                    detector.disable();
                    driveToPosition(-200,-200,200,200,0.2);
                    driveToPosition(1000,-1000,-1000,1000,1);
                    driveToPosition(-2400,-2400,-2400,-2400,1);
                    driveToPosition(500,500,500,500,0.5);
                    driveToPosition(-1000,1000,1000,-1000,1);
                    driveToPosition(-3000,-3000,-3000,-3000,1);
                    driveToPosition(7000,-7000,-7000,7000,1);
                    teamMarker.setPosition(0);
                    sleep(400);
                    driveToPosition(2000,2000,2000,2000,1);
                    driveToPosition(1500,-1500,-1500,1500,1);
                    driveToPosition(8000,8000,8000,8000,1);

                    log.setIsRunning(false);
                    log.closeLogFile();

                }

            }



            detector.disable();
            isRunning = false;


        }

    }
}
