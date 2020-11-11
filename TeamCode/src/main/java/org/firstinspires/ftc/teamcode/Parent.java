package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREVColorDistance;

public abstract class Parent extends LinearOpMode {

    ElapsedTime time = new ElapsedTime();

    DcMotor topLeft;
    DcMotor topRight;
    DcMotor bottomLeft;
    DcMotor bottomRight;
    SensorREVColorDistance frontSensor;


    ColorRangeSensor colorRangeSensor;
    Servo servoLauncher;

    CRServo clawArm;
    CRServo clawElbow;
    CRServo clawClaw;

    public void initRobo(){
        topLeft = hardwareMap.dcMotor.get("topLeftMotor");
        topRight = hardwareMap.dcMotor.get("topRightMotor");
        bottomLeft = hardwareMap.dcMotor.get("bottomLeftMotor");
        bottomRight = hardwareMap.dcMotor.get("bottomRightMotor");

        clawArm = hardwareMap.crservo.get("front arm");
        clawElbow = hardwareMap.crservo.get("back arm");
        clawClaw = hardwareMap.crservo.get("arm servo");

        topLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        topLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        topRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bottomLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        bottomRight.setDirection(DcMotorSimple.Direction.FORWARD);

        clawArm.setDirection(CRServo.Direction.FORWARD);
        clawElbow.setDirection(CRServo.Direction.REVERSE);
        clawClaw.setDirection(CRServo.Direction.FORWARD);

        topLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        topLeft.setPower(0);
        topRight.setPower(0);
        bottomLeft.setPower(0);
        bottomRight.setPower(0);

        clawClaw.setPower(0);
        clawArm.setPower(0);
        clawElbow.setPower(0);

        waitForStart();
        }

        public void setAllRun(){
            topLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            topRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bottomLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bottomRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }

        public void setPowerAll(double pow){
            topLeft.setPower(pow);
            topRight.setPower(pow);
            bottomLeft.setPower(pow);
            bottomRight.setPower(pow);
        }

        public void setPowerAll(double powTL, double powTR, double powBL, double powBR){
            topLeft.setPower(powTL);
            topRight.setPower(powTR);
            bottomLeft.setPower(powBL);
            bottomRight.setPower(powBR);
        }


        public void setPosAll(int pos){
            topLeft.setTargetPosition(pos);
            topRight.setTargetPosition(-pos);
            bottomLeft.setTargetPosition(pos);
            bottomRight.setTargetPosition(-pos);
        }

        public void setPosAll(int posTL, int posTR, int posBL, int posBR){
            topLeft.setTargetPosition(posTL);
            topRight.setTargetPosition(-posTR);
            bottomLeft.setTargetPosition(posBL);
            bottomRight.setTargetPosition(-posBR);
        }



        public void fRbR(int time, double pow) throws InterruptedException{

            setPowerAll(pow);

            Thread.sleep(time);

            setPowerAll(0);
        }

        public void sideways(int time, double pow) throws InterruptedException{

            setPowerAll(pow, -pow, pow, pow);

            Thread.sleep(time);

            setPowerAll(0);
        }

        public void rotation(int time, double pow) throws InterruptedException{

            setPowerAll(pow, -pow, pow, -pow);

            Thread.sleep(time);

            setPowerAll(0);
        }

        public void diagonal(int time, double powLTRB, double powRTLB) throws InterruptedException{

            setPowerAll(powLTRB,powRTLB,powRTLB,powLTRB);

            Thread.sleep(time);

            setPowerAll(0);
        }

       public void clawMove(int time, double pow) throws InterruptedException{
            clawArm.setPower(pow);
            clawElbow.setPower(pow);

            Thread.sleep(time);

            clawArm.setPower(0);
            clawElbow.setPower(0);
       }

       public void openClose(double pow) throws InterruptedException{
            clawClaw.setPower(pow);

            Thread.sleep(1000);

            clawClaw.setPower(0);
       }

       public void moveOpen(int time, double pow) throws InterruptedException{
            clawMove(time, pow);

            openClose(1);

            openClose(-1);
       }
    }


