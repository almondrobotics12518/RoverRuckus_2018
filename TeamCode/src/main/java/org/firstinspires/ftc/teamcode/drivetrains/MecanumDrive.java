package org.firstinspires.ftc.teamcode.drivetrains;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.DcMotor;

public class MecanumDrive {
    private DcMotor lf;
    private DcMotor lb;
    private DcMotor rf;
    private DcMotor rb;

    /* This is the constructor for our mecanum drivetrain
       It takes 4 motors as input.
     */
    public MecanumDrive(DcMotor leftFront, DcMotor leftBack, DcMotor rightFront, DcMotor rightBack){
        lf = leftFront;
        lb = leftBack;
        rf = rightFront;
        rb = rightBack;
    }

    /*
    This method is used to set the power of each of the motors.
     */
    public void setPower(double lfPower,double lbPower, double rfPower, double rbPower){
        lf.setPower(lfPower);
        lb.setPower(lbPower);
        rf.setPower(rfPower);
        rb.setPower(rbPower);
    }

    public void setPowerAll(double power){
        lf.setPower(power);
        lb.setPower(power);
        rf.setPower(power);
        rb.setPower(power);
    }

    public void setEncoderTargetPosition(int lf, int lb, int rf, int rb){
        this.lf.setTargetPosition(lf);
        this.lb.setTargetPosition(lb);
        this.rf.setTargetPosition(rf);
        this.rb.setTargetPosition(rb);
    }


    public void setModeStopAndResetEncoder(){
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setModeRunWithoutEncoder(){
        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setModeRunUsingEncoder(){
        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setModeRunToPosition(){
        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    
}
