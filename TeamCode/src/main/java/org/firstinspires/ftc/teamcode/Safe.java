package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="SafeLander",group="test")
public class Safe extends LinearOpMode {

    DcMotor lScrew;
    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor rightFront;
    DcMotor rightBack;
    double power = 1;

    @Override
    public void runOpMode() throws InterruptedException {

        leftFront = hardwareMap.dcMotor.get("LeftFront");
        leftBack = hardwareMap.dcMotor.get("LeftBack");
        rightFront = hardwareMap.dcMotor.get("RightFront");
        rightBack = hardwareMap.dcMotor.get("RightBack");
        lScrew = hardwareMap.dcMotor.get("LScrew");
        lScrew.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lScrew.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lScrew.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        boolean isRunning = true;
        while (opModeIsActive() && isRunning) {
            lScrew.setPower(-1); // Makes it go down
            sleep(11000); // for 11 seconds1
        }
    }
}
      /*      encoderDrive(500,500,500,500); // Moves towards Outtake side a little bit
            encoderDrive(500, -500, -500, 500); // Moves fowards
            encoderDrive(-600,-600,-600,-600); //  Moves towards Intake side a little bit
            telemetry.addData("Status" ,"Off Hook");
            telemetry.update();
            isRunning = false;

        }
        public void encoderDrive(int lf, int lb, int rf, int rb)
        {
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION); //Runs to the given Position
            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);



            leftFront.setPower(power);
            rightFront.setPower(power); //Runs at the given Power
            leftBack.setPower(power);
            rightBack.setPower(power);

            leftFront.setTargetPosition((leftFront.getCurrentPosition() + lf));
            leftBack.setTargetPosition((leftFront.getCurrentPosition() + lb));
            rightFront.setTargetPosition((leftFront.getCurrentPosition() + rf));
            rightBack.setTargetPosition((leftFront.getCurrentPosition() + rb));

            while(opModeIsActive() &&
                    leftFront.isBusy() && leftBack.isBusy() &&
                    rightFront.isBusy()&& rightBack.isBusy() && !isStopRequested())
            {
                telemetry.addData("Status","moving...");
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

        lScrew.setPower(0);
}

*/