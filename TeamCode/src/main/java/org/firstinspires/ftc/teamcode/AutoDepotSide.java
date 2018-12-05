package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous

public class AutoDepotSide extends LinearOpMode
{

    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;
    private DcMotor lScrew;
    private DcMotor intake;
    BNO055IMU imu;
    private Servo teamMarker;
    private GoldAlignDetector detector;
    private String MineralLocation;

    private double power;

    @Override
    public void runOpMode() throws InterruptedException

    {


        power = 0.8;
        int tarPos = 1500;

        telemetry.addData("Status","Initialize camera.");
        telemetry.update();
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

        telemetry.addData("Status","Detector");
        telemetry.update();
        //This section is the init section. This is where hardware map is put, and motors,
        //servos and sensors are initialized. Here we have hardware map for each of our motors.

        teamMarker = hardwareMap.servo.get("tm");
        leftFront = hardwareMap.dcMotor.get("LeftFront");
        leftBack = hardwareMap.dcMotor.get("LeftBack");
        rightFront = hardwareMap.dcMotor.get("RightFront");
        rightBack = hardwareMap.dcMotor.get("RightBack");
        lScrew = hardwareMap.dcMotor.get("LScrew");
        intake = hardwareMap.dcMotor.get("Intake");

        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lScrew.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lScrew.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Servo stuff in init
        teamMarker.setPosition(1
        );

        waitForStart();

        //Everything after the waitForStart() call is in the start portion of the program.
        //This will run when the user presses the start button.
        lScrew.setPower(-1);
        sleep(7500);
        lScrew.setPower(0);
        telemetry.addData("Status" ,"Done Lowering");
        telemetry.update();

        encoderDrive(-500,-500,-500,-500); // Moves towards Outtake side a little bit
        encoderDrive(500, -500, -500, 500); // Moves fowards
        encoderDrive(600,600,600,600); //  Moves towards Intake side a little bit
        telemetry.addData("Status" ,"Off Hook");
        telemetry.update();


        encoderDrive(tarPos,-tarPos,-tarPos,tarPos);
        telemetry.addData("Status","Moving to minerals");
        telemetry.update();
        if (detector.isFound()){ // Detects if the gold mineral is in the middle
            telemetry.addData("Location","Gold at Middle");
            telemetry.update();
            encoderDrive(4500,-4500,-4500,4500); // Pushes it towards the depot ( sideways )
            MineralLocation = "Middle";

        }
        else {
            encoderDrive(2200,2200,2200,2200); // Goes left ( forwards )
            if (detector.isFound()){ // Detects if the gold mineral is on the left from the hanger
                telemetry.addData("Location","Gold at Intake Side");
                telemetry.update();
                encoderDrive(4500, -4500, -4500, 4500); // Pushes it towards the depot ( sideways )
                encoderDrive(-2000,-2000,-2000,-2000); // Centers in front of the depot
                MineralLocation = "I Side"; //Intake side
            }
            else {
                telemetry.addData("Location","Gold at Outside side");
                telemetry.update();
                encoderDrive(-4500,-4500,-4500,-4500); // Goes right to the last cube ( backwards )
                encoderDrive(4000,-4000,-4000, 4000);  // Pushes it towards the depot ( sideways )
                encoderDrive(2000,2000,2000,2000); // Centers in front of the depot
                MineralLocation = "O side"; //Outtake Side

            }


        }




        encoderDrive(3000,-3000,-3000,3000); //Goes to Depot after gold is moved
        // Deposits Team marker into Depot
        teamMarker.setPosition(0);
        sleep(500);

        encoderDrive(-1500,-1500,1500,1500); //Turns Toward Crater
        encoderDrive(-500,-500,-500,-500);

        encoderDrive(-14000,14000,14000,-14000); // Drive towards the crater



        while(opModeIsActive()){

        }

        telemetry.addData("Status","Robot is in Stop");
        detector.disable(); //Disables camera so it doesn't stay after opmode


    }

    //This method is used for moving the robot around using encoder values. It takes
    //4 encoder values as input for left front, left back, right front, and right back.

    public void encoderDrive(int lf, int lb, int rf, int rb)
    {
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION); //Runs to the given Position
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        leftFront.setPower(power);
        rightFront.setPower(power); //Runs at the given Power
        leftBack.setPower(power);
        rightBack.setPower(power);

        leftFront.setTargetPosition((leftFront.getCurrentPosition() + lf));
        leftBack.setTargetPosition((leftFront.getCurrentPosition() + lb));
        rightFront.setTargetPosition((leftFront.getCurrentPosition() + rf));
        rightBack.setTargetPosition((leftFront.getCurrentPosition() + rb));

        while(opModeIsActive() &&
                leftFront.isBusy() && leftBack.isBusy() &&
                rightFront.isBusy()&& rightBack.isBusy())
        {
            telemetry.addData("Status","moving...");
            telemetry.addData("Left Front Encoder:",leftFront.getCurrentPosition());
            telemetry.addData("Left Back Encoder:",leftBack.getCurrentPosition());
            telemetry.addData("Right Front Encoder:",rightFront.getCurrentPosition());
            telemetry.addData("Right Back Encoder:",rightBack.getCurrentPosition());
            telemetry.addData("isFound",detector.isFound());
            telemetry.update();
        }

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);


        sleep(25);
        


    }
}
