package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="gryotest",group="test")
public class gyroTest extends AlmondLinear {

    public void runOpMode() throws InterruptedException
    {
        telemetry.addData("Status","in init");
        telemetry.update();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        BNO055IMU imu;
        Orientation             lastAngles = new Orientation();
        double globalAngle;

        imu = hardwareMap.get(BNO055IMU.class, "imu");

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

        waitForStart();
        telemetry.addData("Status","in Start");
        telemetry.update();
        while(opModeIsActive())
        {

            Orientation currentAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("First Angle",currentAngles.firstAngle);
            telemetry.addData("First Angle",currentAngles.secondAngle);
            telemetry.addData("First Angle",currentAngles.thirdAngle);
            telemetry.update();
        }

        imu.close();

    }
}
