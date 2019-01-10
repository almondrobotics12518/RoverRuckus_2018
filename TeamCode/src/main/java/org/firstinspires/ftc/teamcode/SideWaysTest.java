package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.drivetrains.DataLog;

@Autonomous(name = "SideWaysTest",group = "test")
public class SideWaysTest extends AlmondLinear {

    public void runOpMode() throws InterruptedException
    {
        DataLogThread2 log;
        hardwareMap();
        log = new DataLogThread2("SideWaysTest",250,leftFront,leftBack,rightFront,rightBack,lScrew);
        log.start();
        waitForStart();
        driveToPosition(5000,-5000,-5000,5000,1);
        log.setIsRunning(false);
        log.closeLogFile();
    }
}
