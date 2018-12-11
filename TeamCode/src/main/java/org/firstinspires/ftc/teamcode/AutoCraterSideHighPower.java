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

@Autonomous(name="CraterSideHighPower",group="AutoCraterSide")
public class AutoCraterSideHighPower extends LinearOpMode
{

    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;
    private DcMotor lScrew;
    BNO055IMU imu;
    private Servo teamMarker;
    private GoldAlignDetector detector;

    private double power;
    boolean isRunning = true;
    @Override
    public void runOpMode() throws InterruptedException

    {
    while (isRunning && opModeIsActive()) {
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

        //This section is the init section. This is where hardware map is put, and motors,
        //servos and sensors are initialized. Here we have hardware map for each of our motors.
        power = 0.5;
        teamMarker = hardwareMap.servo.get("tm");
        leftFront = hardwareMap.dcMotor.get("LeftFront");
        leftBack = hardwareMap.dcMotor.get("LeftBack");
        rightFront = hardwareMap.dcMotor.get("RightFront");
        rightBack = hardwareMap.dcMotor.get("RightBack");
        lScrew = hardwareMap.dcMotor.get("LScrew");

        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lScrew.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lScrew.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        teamMarker.setPosition(1);
        waitForStart();

        //Everything after the waitForStart() call is in the start portion of the program.
        //This will run when the user presses the start button.

        int tarPos0 = 2200;//determines how far the robot goes to sample the minerals
        int tarPos1 = 1500;//amount of encoder values in order to move cube
        int tarPos2 = 5000; // amount it moves between the minerals
        int yeet = 0;//determines how far it needs to go to be relatively centered away from depot
        int b = 0;
        lScrew.setPower(-1);
        sleep(11000);
        lScrew.setPower(0);

        power = 1;
        encoderDrive(500, 500, 500, 500);
        encoderDrive(700, -700, -700, 700);
        encoderDrive(-500, -500, -500, -500);
        //encoderDrive(100,100,-100,-100);

        power = 1;
        telemetry.addData("Status", "Moving to mineral");
        telemetry.update();


        encoderDrive(tarPos0, -tarPos0, -tarPos0, tarPos0);

        if (detector.isFound() && detector.getWidth() > 40) { // If in Middle
            telemetry.addData("Status", "Moving mineral");
            telemetry.update();
            b = -100;
            encoderDrive(tarPos1, -tarPos1, -tarPos1, tarPos1);//travels towards mineral
            encoderDrive(-tarPos1 - 300, tarPos1 + 300, tarPos1 + 300, -tarPos1 - 300);//travels towards mineral some more
        } else {
            encoderDrive(-2200, -2200, -2200, -2200);
            if (detector.isFound() && detector.getWidth() > 40) { // If on Intake side
                yeet = 2200;//set yeet for future instruction
                b = 100;
                telemetry.addData("Status", "Moving mineral on Intake side.");
                telemetry.update();
                encoderDrive(tarPos1, -tarPos1, -tarPos1, tarPos1);//moves mineral
                encoderDrive(-tarPos1 - 300, tarPos1 + 300, tarPos1 + 300, -tarPos1 - 300);//moves back

            } else { // If on Outtake side
                yeet = -2500;//sets yeet for future instruction
                int g = 100;
                b = g;
                telemetry.addData("Status", "Moving mineral on Outtake side.");
                encoderDrive(4400, 4400, 4400, 4400);//goes in front of other mineral
                encoderDrive(tarPos1, -tarPos1, -tarPos1, tarPos1);//moves mineral
                encoderDrive(-tarPos1 - 300, tarPos1 + 300, tarPos1 + 300, -tarPos1 - 300);//travels back

            }
        }
        detector.disable();
        telemetry.addData("Status", "Centering past minerals");
        telemetry.update();
        power = 1;
        encoderDrive(yeet + tarPos2, yeet + tarPos2, yeet + tarPos2, yeet + tarPos2);//moving forwards from minerals
        telemetry.addData("Status", "turning to depot");
        power = 1;// change power
        encoderDrive(-1300 - b, -1300 - b, 1300 + b, 1300 + b);//turns towards depot
        encoderDrive(2000, -2000, -2000, 2000);//moves sideways towards wall
        encoderDrive(8000, 8000, 8000, 8000);//moves into depot

        encoderDrive(-1000, 1000, 1000, -1000);
        teamMarker.setPosition(0);//dumps team marker
        sleep(500);//does nothing for 0.5 seconds
        encoderDrive(-2000, -2000, -2000, -2000);
        encoderDrive(1000, -1000, -1000, 1000);
        encoderDrive(-10000, -10000, -10000, -10000);


        /*encoderDrive(-4100,-4100,4100,4100);//turns towards depot
        telemetry.addData("Status","moving to depot");
        power = 1;//changes power
        encoderDrive(-1100,-1100,-1100,-1100);
        encoderDrive(7000,-7000,-7000,7000);//moves towards depot
        // Deposits Team marker into Depot
        teamMarker.setPosition(0);
        sleep(500);

        encoderDrive(-12000,12000,12000,-12000);*/
        while (opModeIsActive()) {
            if (isStopRequested()) {
                detector.disable();

            }
        }
        detector.disable();
        isRunning = false;
    }
    }

    //This method is used for moving the robot around using encoder values. It takes
    //4 encoder values as input for left front, left back, right front, and right back.

    public void encoderDrive(int lf, int lb, int rf, int rb)
    {
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        leftFront.setPower(power);
        rightFront.setPower(power);
        leftBack.setPower(power);
        rightBack.setPower(power);

        leftFront.setTargetPosition((int) (leftFront.getCurrentPosition() + lf));
        leftBack.setTargetPosition((int) (leftFront.getCurrentPosition() + lb));
        rightFront.setTargetPosition((int) (leftFront.getCurrentPosition() + rf));
        rightBack.setTargetPosition((int) (leftFront.getCurrentPosition() + rb));

        while(opModeIsActive() &&
                leftFront.isBusy() && leftBack.isBusy() &&
                rightFront.isBusy()&& rightBack.isBusy() && !isStarted())
        {
            telemetry.addData("Left Front Encoder:",leftFront.getCurrentPosition());
            telemetry.addData("Left Back Encoder:",leftBack.getCurrentPosition());
            telemetry.addData("Right Front Encoder:",rightFront.getCurrentPosition());
            telemetry.addData("Right Back Encoder:",rightBack.getCurrentPosition());
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

        sleep(50);


    }

}

