package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.AlmondLinear.mineralPosition.MIDDLE;

@Autonomous(name="DepotSide ",group="test")
public class DepotSide extends AlmondLinear
{

    mineralPosition position;

    public void runOpMode() throws InterruptedException
    {
        GoldAlignDetector detector;

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
            detector.maxAreaScorer.weight = 0.005; //

            detector.ratioScorer.weight = 5; //
            detector.ratioScorer.perfectRatio = 1.0; // Ratio adjustment

            detector.enable(); // Start the detector!

            lScrew.setPower(-1);
            sleep(11000);
            lScrew.setPower(0);

            driveToPosition(700,700,700,700,1);
            driveToPosition(1000,-1000,-1000,1000, 1);
            driveToPosition(-1000,-1000,-1000,-1000,1);

            if(detector.isFound()&&detector.getWidth()>40)
            {
                driveToPosition(8000,-8000,-8000,8000,1);
            } else {
                driveToPosition(1000,1000,-1000,-1000,1);
                if(detector.isFound()&&detector.getWidth()>40)
                {
                    driveToPosition(5000,-5000,-5000,5000,1);
                } else {
                    driveToPosition(-2000,-2000,2000,2000,1);
                    driveToPosition(5000,-5000,-5000,5000,1);


                }
            }




            detector.disable();
            stop();
        }

    }
}
