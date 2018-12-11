package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public abstract class AlmondLinear extends LinearOpMode
{

    /*  -------------------------
        Declare drivetrain motors
        ------------------------- */

    DcMotor leftFront;
    DcMotor  rightFront;
    DcMotor leftBack;
    DcMotor rightBack;

    /*  -------------------------------------------
        Declare scoring mechanism servos and motors.
        ------------------------------------------- */

    DcMotor arm;
    DcMotor slide;
    CRServo intake;

    /* -------------------------------------
        Declare hanging lead screw motor.
       ------------------------------------- */

    DcMotor lScrew;

    /*  -------------------------
        Declare team Marker servo.
        ------------------------- */

    Servo teamMarker;

    boolean isRunning = true;
    boolean isAuto = true;
    /*
    This method sets all hardwareMaps for hardware devices
     */
    final public void hardwareMap()
    {

        leftFront = hardwareMap.dcMotor.get("LeftFront");
        leftBack = hardwareMap.dcMotor.get("LeftBack");
        rightFront = hardwareMap.dcMotor.get("RightFront");
        rightBack = hardwareMap.dcMotor.get("RightBack");
        arm = hardwareMap.dcMotor.get("Arm");
        intake = hardwareMap.crservo.get("intake");
        slide = hardwareMap.dcMotor.get("Slide");
        lScrew = hardwareMap.dcMotor.get("LScrew");
        teamMarker = hardwareMap.servo.get("tm");

        if (isAuto)
        {
            leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
            leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
            rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        }
    }


    /*
    This  method is used for moving using encoder values
     */
    final public void driveToPosition(int lf, int lb, int rf,int rb, double power)
    {
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setTargetPosition(leftFront.getCurrentPosition()+lf);
        leftBack.setTargetPosition(lb+leftBack.getCurrentPosition());
        rightFront.setTargetPosition(rf+rightFront.getCurrentPosition());
        rightBack.setTargetPosition(rb+rightBack.getCurrentPosition());

        leftFront.setPower(power);
        leftBack.setPower(power);
        rightBack.setPower(power);
        rightFront.setPower(power);

        while(isRunning && opModeIsActive()
                && leftBack.isBusy() && leftFront.isBusy()
                && rightFront.isBusy() && rightBack.isBusy())
        {
            if(Math.abs(leftFront.getCurrentPosition() - leftFront.getTargetPosition()) < 100 )
            {
                leftFront.setPower(power*0.2);
                leftBack.setPower(power*0.2);
                rightBack.setPower(power*0.2);
                rightFront.setPower(power*0.2);
            }
        }

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);

    }

    public final void setModeAuto() { this.isAuto = true; }

    public final void setModeTeleOp() { this.isAuto = false; }
}

