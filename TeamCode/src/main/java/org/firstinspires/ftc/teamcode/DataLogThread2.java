package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.PrintStream;
import java.text.DateFormat;
import java.util.Date;

/**
 * Created by Luke on 3/29/2017.
 */
public class DataLogThread2 extends Thread {

    private PrintStream logFile;
    private File file;
    private long delay;
    private ElapsedTime currentTime = new ElapsedTime();

    private DcMotor fLD;
    private DcMotor bLD;
    private DcMotor fRD;
    private DcMotor bRD;
    private DcMotor lScrew;

    private int fLEnc;
    private double fLPow;
    private int bLEnc;
    private double bLPow;
    private int fREnc;
    private double fRPow;
    private int bREnc;
    private double bRPow;
    private int lSEnc;
    private double lSPow;
    private boolean isRunning;

    public DataLogThread2(String fileName , long logTime, DcMotor fLD, DcMotor bLD, DcMotor fRD, DcMotor bRD,
                          DcMotor lScrew){

        file = new File(Environment.getExternalStorageDirectory(), createUniqueFileName(fileName, ".txt") + "" );

        try {
            logFile = new PrintStream(new FileOutputStream(file));
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }
        delay = logTime;

        this.fLD = fLD;
        this.bLD = bLD;
        this.fRD = fRD;
        this.bRD = bRD;
        this.lScrew = lScrew;
        isRunning = true;

        fLEnc = 0;
        fLPow = 0.0;
        bLEnc = 0;
        bLPow = 0.0;
        fREnc = 0;
        fRPow = 0.0;
        bREnc = 0;
        bRPow = 0.0;
        lSEnc = 0;
        lSPow = 0.0;

    }

    private String createUniqueFileName(String fileName, String extension){
        return fileName + DateFormat.getDateTimeInstance().format(new Date()) + extension;
    }

    public void run() {

        currentTime.reset();
        logFile.println("time, front left, fl power, back left, bl power, front right, fr power, back right, br power, lScrew, lScrew power");
        while(isRunning){
            saveToTemp();
            logFile.println(currentTime.milliseconds() + ", " + fLEnc + ", " + fRPow + ", " + bLEnc + ", " + bLPow + ", " + fREnc + ", "
                    + fRPow + ", " + bREnc + ", " + bRPow  + ", " + lSEnc + ", " + lSPow  );

            try {
                Thread.sleep(delay);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    private void saveToTemp(){
        int fL = fLD.getCurrentPosition();
        double fLP = fLD.getPower();
        int bL = bLD.getCurrentPosition();
        double bLP = bLD.getPower();
        int fR = fRD.getCurrentPosition();
        double fRP = fRD.getPower();
        int bR = bRD.getCurrentPosition();
        double bRP = bRD.getPower();
        int lSE = lScrew.getCurrentPosition();
        double lSP = lScrew.getPower();



        setfLEnc(fL);
        setfLPow(fLP);
        setbLEnc(bL);
        setbLPow(bLP);
        setfREnc(fR);
        setfRPow(fRP);
        setbREnc(bR);
        setbRPow(bRP);
        setlSPow(lSP);
        setlSEnc(lSE);
    }

    public void setfLEnc(int fLEnc) {this.fLEnc = fLEnc;}

    public void setfLPow(double fLPow) {this.fLPow = fLPow;}

    public void setbLEnc(int bLEnc) {this.bLEnc = bLEnc;}

    public void setbLPow(double bLPow) {this.bLPow = bLPow;}

    public void setfREnc(int fREnc) {this.fREnc = fREnc;}

    public void setfRPow(double fRPow) {this.fRPow = fRPow;}

    public void setbREnc(int bREnc) {this.bREnc = bREnc;}

    public void setbRPow(double bRPow) {this.bRPow = bRPow;}

    public void setlSEnc(int lSEnc) {this.lSEnc = lSEnc;}

    public void setlSPow(double lSPow) {this.lSPow = lSPow;}




    public void setIsRunning(boolean isRunning){this.isRunning = isRunning;}

    public void closeLogFile(){
        logFile.close();
    }
}
