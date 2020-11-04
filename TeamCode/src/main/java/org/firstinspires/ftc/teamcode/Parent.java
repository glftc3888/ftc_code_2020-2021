package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREVColorDistance;

public abstract class Parent extends LinearOpMode {


    DcMotor topLeft;
    DcMotor topRight;
    DcMotor bottomLeft;
    DcMotor bottomRight;
    SensorREVColorDistance frontSensor;


    ColorRangeSensor colorRangeSensor;
    Servo servoLauncher;

    Servo clawArm;
    Servo clawElbow;

    Servo clawClaw;

    public void initRobo(){
        topLeft = hardwareMap.dcMotor.get("topLeftMotor");
        topRight = hardwareMap.dcMotor.get("topRightMotor");
        bottomLeft = hardwareMap.dcMotor.get("bottomLeftMotor");
        bottomRight = hardwareMap.dcMotor.get("bottomRightMotor");

        topLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        topLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        topRight.setDirection(DcMotorSimple.Direction.REVERSE);
        bottomLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        bottomRight.setDirection(DcMotorSimple.Direction.REVERSE);

        topLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        topLeft.setPower(0);
        topRight.setPower(0);
        bottomLeft.setPower(0);
        bottomRight.setPower(0);

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
            topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            bottomRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            bottomLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            topLeft.setTargetPosition(pos);
            topRight.setTargetPosition(pos);
            bottomLeft.setTargetPosition(pos);
            bottomRight.setTargetPosition(pos);
        }

        public void setPosAll(int posTL, int posTR, int posBL, int posBR){
            topLeft.setTargetPosition(posTL);
            topRight.setTargetPosition(posTR);
            bottomLeft.setTargetPosition(posBL);
            bottomRight.setTargetPosition(posBR);
        }



        public void fRbR(int pos, double pow){

            setPosAll(pos, -pos, pos, -pos);

            setPowerAll(pow);

            setAllRun();

            while(topLeft.isBusy()){}

            setPowerAll(0);
        }

        public void sideways(int pos, double pow){
            setPosAll(pos, -pos, -pos, pos);

            setPowerAll(pow);

            setAllRun();

            while(topLeft.isBusy()){}

            setPowerAll(0);
        }

        public void rotation(int pos, double pow){
            setPosAll(-pos, pos, -pos, pos);

            setPowerAll(pow);

            setAllRun();

            while(topLeft.isBusy()){}

            setPowerAll(0);

        }

        public void diagonal(int posLTRB,int posRTLB,double powLTRB, double powRTLB){
            setPosAll(posLTRB,posRTLB,posRTLB,posLTRB);

            setPowerAll(powLTRB,powRTLB,powRTLB,powLTRB);

            setAllRun();

            while(topLeft.isBusy() || topRight.isBusy() ){}

            setPowerAll(0);

        }

    }


