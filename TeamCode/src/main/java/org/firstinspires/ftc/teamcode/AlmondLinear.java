package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static org.firstinspires.ftc.teamcode.AlmondLinear.Direction.FORWARD;

public abstract class AlmondLinear extends LinearOpMode
{


    public int lfEnc = 0;
    public int lbEnc = 0;
    public int rbEnc = 0;
    public int rfEnc = 0;


    public static final int TICKS_PER_REVOLUTION = 1120;
    public static final int WHEEL_DIAMETER_IN_INCHES = 4;
    public static final double TICKS_PER_INCH =TICKS_PER_REVOLUTION/(Math.PI*WHEEL_DIAMETER_IN_INCHES);
    public static final double TICKS_IN_INCH_EMPIRICAL = 0;
    public float currentAngle;
    public float lastAngle;
    // Gyro variable declaration

    public BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    public BNO055IMU imu;

    //dogecv detect declaration


    public GoldAlignDetector detector;

    /*  -------------------------
        Declare drivetrain motors
        ------------------------- */

    public DcMotor leftFront;
    public DcMotor  rightFront;
    public DcMotor leftBack;
    public DcMotor rightBack;

    /*  -------------------------------------------
        Declare scoring mechanism servos and motors.
        ------------------------------------------- */

    public DcMotor arm;
    public DcMotor slide;
    public CRServo intake;

    /* -------------------------------------
        Declare hanging lead screw motor.
       ------------------------------------- */

    public DcMotor lScrew;

    /*  -------------------------
        Declare team Marker servo.
        ------------------------- */

    public Servo teamMarker;

    public boolean isRunning = true;
    public boolean isAuto = true;
    /*
    This method sets all hardwareMaps for hardware devices
     */
    final public void hardwareMap()
    {

        leftFront = hardwareMap.dcMotor.get("LeftFront");
        leftBack = hardwareMap.dcMotor.get("LeftBack");
        rightFront = hardwareMap.dcMotor.get("RightFront");
        rightBack = hardwareMap.dcMotor.get("RightBack");
        intake = hardwareMap.crservo.get("intake");
        lScrew = hardwareMap.dcMotor.get("LScrew");
        teamMarker = hardwareMap.servo.get("tm");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
    The setPower() method sets power to all motors. This is for quality of life to set
    motor powers more easily.
    It takes 4 doubles for each of the 4 drivetrain powers.
     */
    public void setPower(double lf, double lb, double rf, double rb)
    {
        leftFront.setPower(lf);
        leftBack.setPower(lb);
        rightFront.setPower(rf);
        rightBack.setPower(rb);
    }

    public void setPowerAll(double power){
        leftFront.setPower(power);
        leftBack.setPower(power);
        rightFront.setPower(power);
        rightBack.setPower(power);
    }
    /*
    This method sets all the motors to have the same power based on direction that the robot moves.
     */
    public void setPowerDirection(double power, Direction direction)
    {
        switch(direction)
        {
            case FORWARD:
                setPower(power,power,power,power);
                break;
            case BACK:
                setPower(-power,-power,-power,-power);
                break;
            case LEFT:
                setPower(-power,power,power,-power);
                break;
            case RIGHT:
                setPower(power,-power,-power,power);
                break;
        }
    }
    public void updateEncoders(){
        lfEnc = leftFront.getCurrentPosition();
        lbEnc = leftBack.getCurrentPosition();
        rfEnc = rightFront.getCurrentPosition();
        rbEnc = rightBack.getCurrentPosition();
    }
    //This enum is used by setPowerDirection() to select a direction.
    public enum Direction{
        FORWARD,
        BACK,
        LEFT,
        RIGHT
    }
    //This method is incomplete and contains a custom move to position that takes similar input to
    //driveToPosition().
    public void setModeRunUsingEncoders() {
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void gyroTurnSimple(float targetAngle, double power){
        float target = -getCurrentAngle() + 180 + targetAngle;
        while(opModeIsActive()&&-getCurrentAngle()-target>10){
            setPower(power,power,-power,-power);
        }
    }

    public void encoderDrive(int lf,int lb, int rf, int rb, double maxPower){
        setModeRunUsingEncoders();
        double kp = 0.003;
        double ki = 0.000005;
        double kd = 0.0015;
        int integralZone = 200;

        int errorLf;
        int errorLb;
        int errorRf;
        int errorRb;

        int errorTLf=0;
        int errorTLb=0;
        int errorTRf=0;
        int errorTRb=0;

        int lastErrorLf=0;
        int lastErrorLb=0;
        int lastErrorRf=0;
        int lastErrorRb=0;

        double derivativeLf;
        double derivativeLb;
        double derivativeRf;
        double derivativeRb;

        double integralLf;
        double integralRf;
        double integralLb;
        double integralRb;

        double proportionLf;
        double proportionLb;
        double proportionRf;
        double proportionRb;

        double powerLf;
        double powerLb;
        double powerRf;
        double powerRb;

        int startPosLf = leftFront.getCurrentPosition();
        double startPower = 0;

        int targetLf = leftFront.getCurrentPosition()+lf;
        int targetLb = leftBack.getCurrentPosition()+lb;
        int targetRf = rightFront.getCurrentPosition()+rf;
        int targetRb = rightBack.getCurrentPosition()+rb;

        while(opModeIsActive() &&
                Math.abs(leftFront.getCurrentPosition()-targetLf)>10||
                Math.abs(rightFront.getCurrentPosition()-targetRf)>10 ||
                Math.abs(leftBack.getCurrentPosition()-targetLb)>10 ||
                Math.abs(rightBack.getCurrentPosition()-targetRb)>10)
        {
            errorLf = targetLf-leftFront.getCurrentPosition();
            errorLb = targetLb-leftBack.getCurrentPosition();
            errorRf = targetRf-rightFront.getCurrentPosition();
            errorRb = targetRb-rightBack.getCurrentPosition();

            if(Math.abs(errorLf)<integralZone){
                errorTLf += errorLf;
                errorTLb += errorLb;
                errorTRf += errorRf;
                errorTRb += errorRb;
            } else {
                errorTLb = 0;
                errorTLf = 0;
                errorTRf = 0;
                errorTRb = 0;
            }

            proportionLf = errorLf * kp;
            proportionLb = errorLb * kp;
            proportionRb = errorRb * kp;
            proportionRf = errorRf * kp;

            integralLf = errorTLf * ki;
            integralLb = errorTLb * ki;
            integralRf = errorTRf * ki;
            integralRb = errorTRb * ki;

            derivativeLf= -( errorLf - lastErrorLf ) * kd;
            derivativeLb= -( errorLb - lastErrorLb ) * kd;
            derivativeRf= -( errorRf - lastErrorRf ) * kd;
            derivativeRb= -( errorRb - lastErrorRb ) * kd;


            if(Math.abs(errorLf)<5){ derivativeLf = 0; integralLf = 0; }
            if(Math.abs(errorLb)<5){ derivativeLb = 0; integralLb = 0; }
            if(Math.abs(errorRf)<5){ derivativeRf = 0; integralRf = 0; }
            if(Math.abs(errorRb)<5){ derivativeRb = 0; integralRb = 0; }

            if(Math.abs(integralLf)>0.07){ integralLf=(integralLf/Math.abs(integralLf)) * 0.07; }
            if(Math.abs(integralLb)>0.07){ integralLb=(integralLb/Math.abs(integralLb)) * 0.07; }
            if(Math.abs(integralRf)>0.07){ integralRf=(integralRf/Math.abs(integralRf)) * 0.07; }
            if(Math.abs(integralRb)>0.07){ integralRb=(integralRb/Math.abs(integralRb)) * 0.07; }

            powerLf = proportionLf + integralLf + derivativeLf;
            powerLb = proportionLb + integralLb + derivativeLb;
            powerRf = proportionRf + derivativeRf + integralRf;
            powerRb = proportionRb + integralRb + derivativeRb;

            if(Math.abs(powerLf)>1){powerLf/=Math.abs(powerLf);}
            if(Math.abs(powerLf)<0.05){if(powerLf<0){powerLf-=0.05;}else{powerLf+=0.05;}}

            if(Math.abs(powerLb)>1){powerLb/=Math.abs(powerLb);}
            if(Math.abs(powerLb)<0.05){if(powerLb<0){powerLb-=0.05;}else{powerLb+=0.05;}}

            if(Math.abs(powerRf)>1){powerRf/=Math.abs(powerRf);}
            if(Math.abs(powerRf)<0.05){if(powerRf<0){powerRf-=0.05;}else{powerRf+=0.05;}}

            if(Math.abs(powerRb)>1){powerRb/=Math.abs(powerRb);}
            if(Math.abs(powerRb)<0.05){if(powerRb<0){powerRb-=0.05;}else{powerRb+=0.05;}}

            setPower(powerLf,powerLb,powerRf,powerRb);

            telemetry.addData("Proportion",proportionLf);
            telemetry.addData("Encoder",leftFront.getCurrentPosition());
            telemetry.addData("Target",targetLf);
            telemetry.addData("Power",leftFront.getPower());
            telemetry.addData("error",errorLf);
            telemetry.update();
        }
        setPowerAll(0);

    }

    /*
    This  method is used for moving using encoder values. It takes 4 encoder values for the 4
    drivetrain motors and moves to target position. It also takes a power value of
     */
    final public void driveToPosition(int lf, int lb, int rf,int rb, double power) {

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setTargetPosition(leftFront.getCurrentPosition() + lf);
        leftBack.setTargetPosition(lb + leftBack.getCurrentPosition());
        rightFront.setTargetPosition(rf + rightFront.getCurrentPosition());
        rightBack.setTargetPosition(rb + rightBack.getCurrentPosition());

        while (opModeIsActive() &&
                leftBack.isBusy() && leftFront.isBusy()
                && rightFront.isBusy() && rightBack.isBusy())
        {


            power = (leftFront.getCurrentPosition()-leftFront.getTargetPosition())/1000;
            if(power>1){
                power = 1;
            }
            setPowerAll(power);

        }
    }


    public void driveLeftMotor(int target){
        int tarLf = lfEnc + target;
        int errorLf = tarLf - leftFront.getCurrentPosition();

        while(errorLf>5 && opModeIsActive()){
            leftFront.setPower(1);
        }
        while(errorLf<5 && opModeIsActive()){
            leftFront.setPower(-1);
        }
        leftFront.setPower(0);
    }


    public final void detectorEnable()
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
    }
    
    //Method initializes the imu sensor.
    public void initImu()
    {

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu.initialize(parameters);
        telemetry.addData("Status","Calibrating...");
        telemetry.update();

        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }


        telemetry.addData("Status","Done calibrating. Waiting for start.");
        telemetry.update();
    }



    //This method is currently unused. Work in progress turn based on gyro angle.
    public float getCurrentAngle()
    {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle + 180;
    }

    public final void setModeAuto() { this.isAuto = true; }

    public final void setModeTeleOp() { this.isAuto = false; }

    //Unused enum for position of mineral in sampling.
    public enum mineralPosition {
        LEFT,
        RIGHT,
        MIDDLE,
        UNKNOWN,
    }
    //This method resets all encoders that are used.
    public void resetEncoders(){
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lScrew.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
}

