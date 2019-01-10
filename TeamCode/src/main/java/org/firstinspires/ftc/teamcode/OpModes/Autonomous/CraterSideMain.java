package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AlmondLinear;
import org.firstinspires.ftc.teamcode.DataLogThread2;

@Autonomous(name="CraterSide",group="opmodes")
public class CraterSideMain extends AlmondLinear
{

    mineralPosition position;


    public void runOpMode() throws InterruptedException
    {
        GoldAlignDetector detector;
        DataLogThread2 log;




        hardwareMap();
        teamMarker.setPosition(1);
        waitForStart();
        while (opModeIsActive() && isRunning)
        {


            // Set up detector
            detector = new GoldAlignDetector(); // Create detector
            detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance()); // Initialize it with the app context and camera
            detector.useDefaults(); // Set detector to use default settings

            // Optional tuning
            detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
            detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
            detector.downscale = 0.4; // How much to downscale the input frames

            detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
            //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
            detector.maxAreaScorer.weight = 0.005;

            detector.ratioScorer.weight = 5;
            detector.ratioScorer.perfectRatio = 1.0; // Ratio adjustment

            detector.enable(); // Start the detector!

            //lScrew.setPower(-1); // Robot lowers
            //sleep(11000);
            //lScrew.setPower(0);

            driveToPosition(700,700,700,700,1); // Moves forward
            driveToPosition(500,-500,-500,500, 1); // Moves towards cube ( right )
            driveToPosition(-900,-900,-900,-900,1); // Moves backward

            if(detector.isFound()&&detector.getWidth()>40)
            {
                detector.disable();
                telemetry.addData("Status","Pushing Middle Cube...");
                telemetry.update();
                driveToPosition(-200,-200,-200,-200,1);
                driveToPosition(4400,-4400,-4400,4400,1); // Moves towards cube ( right )
                driveToPosition(-2000,2000,2000,-2000,1);
                driveToPosition(7400,7400,7400,7400,1);
                driveToPosition(-1300,-1300,1300,1300,1);
                driveToPosition(4000,4000,4000,4000,1);
                driveToPosition(-2700,-2700,2700,2700,1);
                teamMarker.setPosition(0);
                driveToPosition(-3200,-3000,3200,3200,1);
                driveToPosition(10000,10000,10000,10000,1);



            } else {
                driveToPosition(1300,1300,-1300,-1300,1); // Moves clockwise

                if(detector.isFound() &&detector.getWidth()>40)
                {
                    detector.disable();
                    telemetry.addData("Status","Pushing Outtake Side Cube...");
                    telemetry.update();

                    driveToPosition(5200,-5200,-5200,5200,1); // Moves towards cube ( right )
                    driveToPosition(-3700,3700,3700,-3700,1);
                    driveToPosition(-700,-700,700,700,1);
                    driveToPosition(6600,6600,6600,6600,1);
                    driveToPosition(4200,4200,-4200,-4200,1);
                    driveToPosition(-7700,-7700,-7700,-7700,1);
                    teamMarker.setPosition(0);
                    sleep(500);
                    driveToPosition(11000,11000,11000,11000,1);


                } else {
                    detector.disable();
                    telemetry.addData("Status","Pushing Intake Side Cube...");
                    telemetry.update();
                    driveToPosition(-2200,-2200,2200,2200,1); // Moves counterclockwise
                    driveToPosition(5500,-5500,-5500,5500,1); // Moves towards cube ( right// )
                    driveToPosition(-1900,1900,1900,-1900,1);
                    driveToPosition(500,500,-500,-500,1);
                    driveToPosition(6000,6000,6000,6000,1);
                    driveToPosition(-700,-700,700,700,1);
                    driveToPosition(2000,2000,2000,2000,1);
                    driveToPosition(-2700,-2700,2700,2700,1);
                    teamMarker.setPosition(0);
                    driveToPosition(-3200,-3000,3200,3200,1);
                    driveToPosition(10000,10000,10000,10000,1);
                }
            }



            detector.disable();
            isRunning = false;
            stop();
        }

    }
}