package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

abstract class Parent extends OpMode {


    DcMotor topLeft;
    DcMotor topRight;
    DcMotor bottomLeft;
    DcMotor bottomRight;

    public void init(){
        topLeft = hardwareMap.dcMotor.get("Top left motor");
        topRight = hardwareMap.dcMotor.get("Top right motor");
        bottomLeft = hardwareMap.dcMotor.get("Bottom left motor");
        bottomRight = hardwareMap.dcMotor.get("Bottom right motor");

        topLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        topLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        topRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bottomLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        bottomRight.setDirection(DcMotorSimple.Direction.FORWARD);

        topLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        topLeft.setPower(0);
        topRight.setPower(0);
        bottomLeft.setPower(0);
        bottomRight.setPower(0);
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
            setPosAll(pos);

            setPowerAll(pow);

            while(topLeft.isBusy()){}

            setPowerAll(0);
        }

        public void sideways(int pos, double pow){
            setPosAll(pos, -pos, -pos, pos);

            setPowerAll(pow);

            while(topLeft.isBusy()){}

            setPowerAll(0);
        }

        public void rotation(int pos, double pow){
            setPosAll(-pos, pos, -pos, pos);

            setPowerAll(pow);

            while(topLeft.isBusy()){}

            setPowerAll(0);

        }
        

    }


