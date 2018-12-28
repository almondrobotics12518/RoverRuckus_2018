package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="AutoCraterSide",group="test")
public class AutoCraterSide extends AlmondLinear
{

    mineralPosition position;


    public void runOpMode() throws InterruptedException
    {
        GoldAlignDetector detector;
        DataLogThread2 log;
        log = new DataLogThread2("CraterLog",250,leftFront,leftBack,rightFront,rightBack,lScrew);
        hardwareMap();
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
            driveToPosition(-700,-700,-700,-700,1); // Moves backward

            if(detector.isFound()&&detector.getWidth()>40)
            {
                detector.disable();
                telemetry.addData("Status","Pushing Middle Cube...");
                telemetry.update();
                driveToPosition(4000,-8000,-4000,4000,1); // Moves towards cube ( right )
                driveToPosition(-2000,2000,2000,-2000,1);
                driveToPosition(3000,3000,3000,3000,1);

            } else {
                driveToPosition(1200,1200,-1200,-1200,1); // Moves clockwise

                if(detector.isFound() &&detector.getWidth()>40)
                {
                    detector.disable();
                    telemetry.addData("Status","Pushing Outtake Side Cube...");
                    telemetry.update();
                    driveToPosition(4300,-4300,-4300,4300,1); // Moves towards cube ( right )
                    driveToPosition(-3500,3500,3500,-3500,1);
                    driveToPosition(4000,4000,4000,4000,1);


                } else {
                    detector.disable();
                    telemetry.addData("Status","Pushing Intake Side Cube...");
                    telemetry.update();
                    driveToPosition(-2000,-2000,2000,2000,1); // Moves counterclockwise
                    driveToPosition(4000,-4000,-4000,4000,1); // Moves towards cube ( right )

                }
            }

            driveToPosition(1000,1000,-1000,-1000,1);

            log.setIsRunning(false);
            log.closeLogFile();
            detector.disable();
            isRunning = false;
            stop();
        }

    }
}
