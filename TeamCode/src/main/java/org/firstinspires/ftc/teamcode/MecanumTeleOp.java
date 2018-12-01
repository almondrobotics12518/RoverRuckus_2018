package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.sun.source.tree.BreakTree;

@TeleOp(name="MecanumTeleOp",group="mecanum")
public class MecanumTeleOp extends OpMode {

    private DcMotor lScrew;
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotor slide;
    private DcMotor arm;
    private float leftX;
    private float leftY;
    private float rightX;
    private GoldAlignDetector detector;
    private double LF;
    private double RF;
    private double LB;
    private double RB;
   ;

    @Override
    public void init(){


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
        // Setting dcMotor variables to motors
        leftFront = hardwareMap.dcMotor.get("LeftFront");
        leftBack = hardwareMap.dcMotor.get("LeftBack");
        rightFront = hardwareMap.dcMotor.get("RightFront");
        rightBack = hardwareMap.dcMotor.get("RightBack");
        lScrew = hardwareMap.dcMotor.get("LScrew");
        slide = hardwareMap.dcMotor.get("Slide");
        arm = hardwareMap.dcMotor.get("Arm");

        // Reversing direction of right side motors
        //leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        //rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        /*leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lScrew.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/
    }

    @Override
    public void loop(){
        leftX = -gamepad1.left_stick_x; // Reverse left joystick's X coordinate
        leftY = gamepad1.left_stick_y; // Reverse left joystick's Y coordinate
        rightX = -gamepad1.right_stick_x;
        double speed = Math.hypot(leftX, leftY); // Takes hypotenuse of leftX and leftY
        double angle = Math.atan2(leftY, leftX) - Math.PI / 4; // Calculates angle of direction

        LF = speed * Math.cos(angle)+rightX; // Calculates power for moving for the LF wheel
        LB = speed * Math.sin(angle)+rightX; // Calculates power for moving for the LB wheel
        RF = speed * Math.sin(angle)-rightX; // Calculates power for moving for the RF wheel
        RB = speed * Math.cos(angle)-rightX; // Calculates power for moving for the RB wheel

        leftFront.setPower(LF); // Gives power to LF wheels
        leftBack.setPower(LB); // Gives power to LB wheels
        rightFront.setPower(RF); // Gives power to RF wheels
        rightBack.setPower(RB); // Gives power to RB wheels
        lScrew.setPower(gamepad1.right_trigger-gamepad1.left_trigger); // Gives power to the lScrew
        arm.setPower(-gamepad2.right_stick_y * 0.50); // Gives power to the arm
        slide.setPower(-gamepad2.left_stick_y); // Gives power to the slide
        // Add telemetry variables and updating them
        telemetry.addData("FrontLeftPower",LF);
        telemetry.addData("FrontRightPower",RF);
        telemetry.addData("BackLeftPower",LB);
        telemetry.addData("BackRightPower",RB);
        telemetry.addData("Detector isFound", detector.isFound());
        telemetry.addData("Encoders --","------");
        telemetry.addData("LeftFront",leftFront.getCurrentPosition());
        telemetry.addData("LeftBack",leftBack.getCurrentPosition());
        telemetry.addData("RightFront",rightFront.getCurrentPosition());
        telemetry.addData("RightBack",rightBack.getCurrentPosition());
        telemetry.addData("Hang",lScrew.getCurrentPosition());
        telemetry.addData("Arm zeroPowerBehavior",arm.getZeroPowerBehavior());
        telemetry.update();


    }
    @Override
    public void stop(){

    }
}