package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREVColorDistance;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


abstract public class Parent2 extends LinearOpMode {

    ElapsedTime time = new ElapsedTime();

    //DT
    DcMotor topLeft;
    DcMotor topRight;
    DcMotor bottomLeft;
    DcMotor bottomRight;

    DcMotor intake;

    DcMotor launchLeft;
    DcMotor launchRight;

    RevColorSensorV3 tSense;
    RevColorSensorV3 bSense;

    Servo clawGrab;
    Servo fPin;
    CRServo rAP;
    CRServo wrist;
    double powerBase;
    /*
    //Gyro
    BNO055IMU imu;
    Orientation angles;
    */

    //initalizes Robot
    public void initRobo() {
        topLeft = hardwareMap.dcMotor.get("topLeftMotor");
        topRight = hardwareMap.dcMotor.get("topRightMotor");
        bottomLeft = hardwareMap.dcMotor.get("bottomLeftMotor");
        bottomRight = hardwareMap.dcMotor.get("bottomRightMotor");
        intake = hardwareMap.dcMotor.get("intake");

        clawGrab = hardwareMap.servo.get("grab");
        wrist = hardwareMap.crservo.get("wrist");
        rAP = hardwareMap.crservo.get("rAP");
        fPin = hardwareMap.servo.get("fPin");

        tSense = hardwareMap.get(RevColorSensorV3.class, "tSense");
        bSense = hardwareMap.get(RevColorSensorV3.class, "bSense");

        topLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        topLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        topRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bottomLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        bottomRight.setDirection(DcMotorSimple.Direction.FORWARD);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        clawGrab.setDirection(Servo.Direction.FORWARD);
        wrist.setDirection(CRServo.Direction.FORWARD);
        rAP.setDirection(CRServo.Direction.FORWARD);
        fPin.setDirection(Servo.Direction.FORWARD);

        topLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        topLeft.setPower(0);
        topRight.setPower(0);
        bottomLeft.setPower(0);
        bottomRight.setPower(0);


        rAP.setPower(0);
        wrist.setPower(0);

        fPin.setPosition(0.5);

        waitForStart();

    }


    //304.8mm = 1 foot, 1440 ticks = 100mm, 4,389 ticks = 1 foot
    // distance = +70 for 1 tile (to front of bot)
    public void move(double power, int distance) {
        /*
        topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //bottomLeft.setMode(topLeft.getMode());
        //bottomRight.setMode(topRight.getMode());
        */
        topRight.setTargetPosition(-distance);
        topLeft.setTargetPosition(-distance);
        bottomRight.setTargetPosition(-distance);
        bottomLeft.setTargetPosition(-distance);

        topLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        topRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottomLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottomRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        topRight.setPower(power);
        topLeft.setPower(power);
        bottomRight.setPower(power);
        bottomLeft.setPower(power);

        outerM: while(true){
            if(Math.abs(topLeft.getCurrentPosition()) >= Math.abs(distance)){
                break outerM;
            }
        }

        telemetry.addLine("The while loop broke.");
        topRight.setPower(0);
        topLeft.setPower(0);
        bottomRight.setPower(0);
        bottomLeft.setPower(0);

    }

    // +power = right, -power = Left
    // power = 0.125
    // distance = +70 for a tile (to right)
    public void moveSideways(double power, int distance){
        /*
        topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        */
        topRight.setTargetPosition(-distance);
        topLeft.setTargetPosition(distance);
        bottomRight.setTargetPosition(distance);
        bottomLeft.setTargetPosition(-distance);


        topLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        topRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottomLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottomRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        topRight.setPower(power);
        topLeft.setPower(power);
        bottomRight.setPower(power);
        bottomLeft.setPower(power);


        outerMS: while(topLeft.isBusy() && topRight.isBusy() && bottomLeft.isBusy() && bottomRight.isBusy()){
            telemetry.addData("tL pos ", topLeft.getCurrentPosition());
            telemetry.addData("tR pos ", topRight.getCurrentPosition());
            telemetry.addData("bL pos ", bottomLeft.getCurrentPosition());
            telemetry.addData("bR pos ", bottomRight.getCurrentPosition());
            telemetry.update();
            if(!(topLeft.isBusy() && topRight.isBusy() && bottomLeft.isBusy() && bottomRight.isBusy())){
                break outerMS;
            }
        }
        topRight.setPower(0);
        topLeft.setPower(0);
        bottomRight.setPower(0);
        bottomLeft.setPower(0);
    }

    // turns,
    // distance = +800 for 1 complete 360 rotation (to left)
    public void turn ( double power, int distance){
        /*
        topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        */
        topRight.setTargetPosition(distance);
        topLeft.setTargetPosition(-distance);
        bottomRight.setTargetPosition(distance);
        bottomLeft.setTargetPosition(-distance);


        topLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        topRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottomLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottomRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        topRight.setPower(power);
        topLeft.setPower(power);
        bottomRight.setPower(power);
        bottomLeft.setPower(power);


        outerT: while(true) {
            if(Math.abs(topLeft.getCurrentPosition()) >= Math.abs(distance)){
                break outerT;
            }
        }

        topRight.setPower(0);
        topLeft.setPower(0);
        bottomRight.setPower(0);
        bottomLeft.setPower(0);
    }
    /*
        Color Sensor Range normal: 100 to 300
        Color Sensor Range with ring: 4000 to 5000
        auto1 = bSense false
        auto2 = bSense true, tSense false
        auto3 = bSense true, tSense true
    */
    public void moveRacPin(int time, double pow) throws InterruptedException {
        rAP.setPower(-pow);

        Thread.sleep(time);

        rAP.setPower(0);
    }

    public void moveWrist(int time, double pow) throws InterruptedException {
        wrist.setPower(pow);

        Thread.sleep(time);

        wrist.setPower(0);
    }

    public void moveGrip(double pos) throws InterruptedException {
        clawGrab.setPosition(pos);

        Thread.sleep(1000);
    }
}