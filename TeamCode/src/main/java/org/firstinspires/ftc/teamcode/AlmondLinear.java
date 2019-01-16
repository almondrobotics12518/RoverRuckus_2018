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
        arm = hardwareMap.dcMotor.get("Arm");
        intake = hardwareMap.crservo.get("intake");
        slide = hardwareMap.dcMotor.get("Slide");
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

    public void PIDdriveToPosition(double target, double kp, double ki, double kd){
        double integralZone = 500;
        double power = 0;
        boolean isMoving = true;
        double targetPosition = 0;
        double error = 0;
        double proportion = 0;
        double integral = 0;
        double derivative = 0;
        double errorT = 0;
        double lastError = 0;
        while(opModeIsActive()&&isMoving) {
            targetPosition = leftFront.getCurrentPosition() + target;
            error = targetPosition - leftFront.getCurrentPosition();

            if (error < integralZone && error != 0) {
                errorT += error;
            } else {
                errorT = 0;
            }
            if (errorT > 50 / ki) {
                errorT = 50 / ki;
            }
            if (error == 0) {
                integral = 0;
            }


            proportion = error * kp;
            integral = errorT * ki;
            derivative = (error - lastError)*kd;

            power = proportion + integral + derivative;
            if(power>1){
                power = 1;
            }
            setPowerAll(power);
            if(power ==0){
                isMoving = false;
            }

            sleep(20);


        }
    }

    public void encoderDrive(int lf){
        setModeRunUsingEncoders();
        double power = 0;
        int startLf = leftFront.getCurrentPosition();
        int targetLf = leftFront.getCurrentPosition()+lf;
        leftFront.setPower(1);
        while(opModeIsActive() && leftFront.getCurrentPosition()<targetLf){
            power = (leftFront.getCurrentPosition()-startLf)/1000;
            if (power>1){ power = 1; }
            leftFront.setPower(power);
            sleep(20);
        }
        leftFront.setPower(0);
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

        sleep(1000);

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
    public void driveForwardDistance(double inches){
        int targetPosition = (int)(inches*TICKS_PER_INCH);
        driveToPosition(targetPosition,targetPosition,targetPosition,targetPosition,1);
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

