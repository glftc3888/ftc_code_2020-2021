package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

    DcMotor topLeft;
    DcMotor topRight;
    DcMotor bottomLeft;
    DcMotor bottomRight;
    DcMotor intake;
    SensorREVColorDistance frontSensor;
    BNO055IMU Gyro;

    DcMotor launchLeft;
    DcMotor launchRight;

    ColorRangeSensor colorRangeSensor;
    Servo servoLauncher;

    Servo clawGrab;
    Servo fPin;
    CRServo rAP;
    CRServo wrist;
    double powerBase;

    public void initRobo() {
        topLeft = hardwareMap.dcMotor.get("topLeftMotor");
        topRight = hardwareMap.dcMotor.get("topRightMotor");
        bottomLeft = hardwareMap.dcMotor.get("bottomLeftMotor");
        bottomRight = hardwareMap.dcMotor.get("bottomRightMotor");
        intake = hardwareMap.dcMotor.get("intake");

        launchLeft = hardwareMap.dcMotor.get("launchLeftMotor");
        launchRight = hardwareMap.dcMotor.get("launchRightMotor");

        clawGrab = hardwareMap.servo.get("grab");
        wrist = hardwareMap.crservo.get("wrist");
        rAP = hardwareMap.crservo.get("rAP");
        fPin = hardwareMap.servo.get("fPin");


        Gyro = hardwareMap.get(BNO055IMU.class, "Gyro");
        Orientation orient = new Orientation();
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        Gyro.initialize(parameters);
        telemetry.addData("Calibrating Gyro: ", Gyro.getCalibrationStatus().toString());
        telemetry.addData("Gyro ready?: ", Gyro.isGyroCalibrated());


        topLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        topLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        topRight.setDirection(DcMotorSimple.Direction.REVERSE);
        bottomLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        bottomRight.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        launchLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launchRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        clawGrab.setDirection(Servo.Direction.FORWARD);
        wrist.setDirection(CRServo.Direction.FORWARD);
        rAP.setDirection(CRServo.Direction.FORWARD);
        fPin.setDirection(Servo.Direction.FORWARD);

        launchLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        launchRight.setDirection(DcMotorSimple.Direction.FORWARD);

        topLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        topLeft.setPower(0);
        topRight.setPower(0);
        bottomLeft.setPower(0);
        bottomRight.setPower(0);

        launchLeft.setPower(0);
        launchRight.setPower(0);


        rAP.setPower(0);
        wrist.setPower(0);

        launchLeft.setPower(0);
        launchRight.setPower(0);

        fPin.setPosition(0.5);
        clawGrab.setPosition(1);

        topRight.setMode(bottomLeft.getMode());
        topLeft.setMode(bottomLeft.getMode());
        bottomRight.setMode(bottomLeft.getMode());

        launchLeft.setMode(launchRight.getMode());
        powerBase = 12/this.hardwareMap.voltageSensor.iterator().next().getVoltage();
        waitForStart();

    }


    //304.8mm = 1 foot, 1440 ticks = 100mm, 4,389 ticks = 1 foot
    public void move(double power, int distance) {
        topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        topLeft.setTargetPosition(-distance);
        topRight.setTargetPosition(-distance);
        bottomLeft.setTargetPosition(-distance);
        bottomRight.setTargetPosition(-distance);

        topLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        topRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottomLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottomRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        topRight.setPower(power);
        topLeft.setPower(-power);
        bottomRight.setPower(power);
        bottomLeft.setPower(-power);

        while (topLeft.isBusy() && bottomLeft.isBusy() && topRight.isBusy() && bottomRight.isBusy());

        topRight.setPower(0);
        topLeft.setPower(0);
        bottomRight.setPower(0);
        bottomLeft.setPower(0);

    }

    // +power = right, -power = Left
    public void moveSideways(double power, int distance) {
        topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        topLeft.setTargetPosition(distance);
        topRight.setTargetPosition(-distance);
        bottomLeft.setTargetPosition(-distance);
        bottomRight.setTargetPosition(distance);

        topLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        topRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottomLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottomRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        topRight.setPower(power);
        topLeft.setPower(-power);
        bottomRight.setPower(-power);
        bottomLeft.setPower(power);


        while (topLeft.isBusy() && bottomLeft.isBusy() && topRight.isBusy() && bottomRight.isBusy()) {

        }


        topRight.setPower(0);
        topLeft.setPower(0);
        bottomRight.setPower(0);
        bottomLeft.setPower(0);

    }

    // turns,
    public void turn ( double power, int distance){
        topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        topLeft.setTargetPosition(-distance);
        topRight.setTargetPosition(distance);
        bottomLeft.setTargetPosition(-distance);
        bottomRight.setTargetPosition(distance);

        topLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        topRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottomLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottomRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        topRight.setPower(-power);
        topLeft.setPower(power);
        bottomRight.setPower(-power);
        bottomLeft.setPower(power);

        while (Math.abs(topLeft.getCurrentPosition()) < Math.abs(topLeft.getTargetPosition()));

        topRight.setPower(0);
        topLeft.setPower(0);
        bottomRight.setPower(0);
        bottomLeft.setPower(0);
    }
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

}
