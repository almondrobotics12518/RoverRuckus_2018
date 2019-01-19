package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name="LinearTeleOp",group="opmodes")
public class TeleOpMain extends LinearOpMode {

    private DcMotor lScrew;
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotor slide;
    private DcMotor armLeft;
    private DcMotor armRight;
    private CRServo intake;
    private float leftX;
    private float leftY;
    private float rightX;
    private double LF;
    private double RF;
    private double LB;
    private double RB;
    private double armY;

    @Override
    public void runOpMode() throws InterruptedException{
        // Setting dcMotor variables to motors
        leftFront = hardwareMap.dcMotor.get("LeftFront");
        leftBack = hardwareMap.dcMotor.get("LeftBack");
        rightFront = hardwareMap.dcMotor.get("RightFront");
        rightBack = hardwareMap.dcMotor.get("RightBack");
        armLeft = hardwareMap.dcMotor.get("ArmLeft");
        armRight = hardwareMap.dcMotor.get("ArmRight");
        intake = hardwareMap.crservo.get("intake");
        slide = hardwareMap.dcMotor.get("Slide");
        lScrew = hardwareMap.dcMotor.get("LScrew");

        // Reversing direction of right side motors
        //leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        //rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        armLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        /*leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lScrew.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/

        // dont need isrunning
        waitForStart();
        while(opModeIsActive()){
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

            armY = gamepad2.right_stick_y*0.25;
            if (Math.abs(armY)==0){
                armLeft.setPower(0);
                armRight.setPower(0);
            } else {
                armY=(0.2*armY/Math.abs(armY))+armY;
                armLeft.setPower(-armY); // Gives power to the arm
                armRight.setPower(armY);
            }




            intake.setPower(gamepad2.right_trigger-gamepad2.left_trigger); //Spins the Intake

            slide.setPower(-gamepad2.left_stick_y*0.5); // Gives power to the slide

            // Add telemetry variables and updating them
            telemetry.addData("FrontLeftPower",LF);
            telemetry.addData("FrontRightPower",RF);
            telemetry.addData("BackLeftPower",LB);
            telemetry.addData("BackRightPower",RB);
            telemetry.addData("Encoders --","------");
            telemetry.addData("LeftFront",leftFront.getCurrentPosition());
            telemetry.addData("LeftBack",leftBack.getCurrentPosition());
            telemetry.addData("RightFront",rightFront.getCurrentPosition());
            telemetry.addData("RightBack",rightBack.getCurrentPosition());
            telemetry.addData("Hang",lScrew.getCurrentPosition());
            telemetry.addData("ArmLeft zeroPowerBehavior",armLeft.getZeroPowerBehavior());
            telemetry.addData("ArmRight zeroPowerBehavior", armRight.getZeroPowerBehavior());
            telemetry.addData("ArmRight Power",armRight.getPower());
            telemetry.addData("ArmLeft Power",armLeft.getPower());
            telemetry.addData("Slide",slide.getCurrentPosition());
            telemetry.addData("Arm y",armY);
            telemetry.addData("Gamepad 2 left stick Y",gamepad2.left_stick_y);
            telemetry.update();

            // dont need this

        }
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        slide.setPower(0);
        armLeft.setPower(0);
        armRight.setPower(0);
        lScrew.setPower(0);



    }
}
