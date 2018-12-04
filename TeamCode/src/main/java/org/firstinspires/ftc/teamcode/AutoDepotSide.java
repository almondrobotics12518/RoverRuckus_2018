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

@Autonomous(name="DepotSide0",group="AutoDepotSide")
public class AutoDepotSide extends LinearOpMode
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

    @Override
    public void runOpMode() throws InterruptedException

    {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

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
        int tarPos = 2000;
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

        teamMarker.setPosition(1);

        waitForStart();

        //Everything after the waitForStart() call is in the start portion of the program.
        //This will run when the user presses the start button.
        lScrew.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lScrew.setPower(-1);
        sleep(8000);
        lScrew.setPower(0);

        encoderDrive(-500,-500,-500,-500);
        encoderDrive(500, -500, -500, 500);
        encoderDrive(500,500,500,500);

        encoderDrive(tarPos,-tarPos,-tarPos,tarPos);
        if (detector.isFound()){ // Detects if the gold mineral is in the middle
            encoderDrive(5000,-5000,-5000,5000); // Pushes it towards the depot ( sideways )

        }
        else {
            encoderDrive(tarPos,tarPos,tarPos,tarPos); // Goes left ( forwards )
            if (detector.isFound()){ // Detects if the gold mineral is on the left from the hanger
                encoderDrive(5000, -5000, -5000, 5000); // Pushes it towards the depot ( sideways )
                encoderDrive(-2000,-2000,-2000,-2000);

            }
            else {
                encoderDrive(-4000,-4000,-4000,-4000); // Goes right to the last cube ( backwards )
                encoderDrive(5000,-5000,-5000, 5000);  // Pushes it towards the depot ( sideways )
                encoderDrive(2000,2000,2000,2000);
            }
        }



        encoderDrive(4000,-4000,-4000,4000);
        teamMarker.setPosition(0);
        encoderDrive(500,500,-500,-500);



        while(opModeIsActive()){

        }

        detector.disable();


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
                rightFront.isBusy()&& rightBack.isBusy())
        {
            telemetry.addData("Left Front Encoder:",leftFront.getCurrentPosition());
            telemetry.addData("Left Back Encoder:",leftBack.getCurrentPosition());
            telemetry.addData("Right Front Encoder:",rightFront.getCurrentPosition());
            telemetry.addData("Right Back Encoder:",rightBack.getCurrentPosition());
            telemetry.addData("Hang", lScrew.getCurrentPosition());
            telemetry.addData("isFound",detector.isFound());
            telemetry.update();
        }
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);


        sleep(50);

    }

    }

