package org.firstinspires.ftc.teamcode.subsystems.drivetrains;

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

    public void setMode(DcMotor.RunMode mode){
        switch(mode){
            case RUN_TO_POSITION:
                lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                break;
            case RUN_USING_ENCODER:
                lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                break;
            case RUN_WITHOUT_ENCODER:
                lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            case STOP_AND_RESET_ENCODER:
                lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        }
    }


}
