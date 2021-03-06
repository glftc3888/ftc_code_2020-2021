package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREVColorDistance;

public abstract class Parent extends LinearOpMode {

    ElapsedTime time = new ElapsedTime();

    DcMotor topLeft;
    DcMotor topRight;
    DcMotor bottomLeft;
    DcMotor bottomRight;
    DcMotor intake;

    DcMotor launchLeft;
    DcMotor launchRight;
    Servo fPin;
    /*
    RevColorSensorV3 tSense;
    RevColorSensorV3 bSense;


    Servo clawGrab;
    CRServo rAP;
    CRServo wrist;
    double powerBase;

     */
    BNO055IMU imu;
    Orientation angles;
    double max = 0.9;


    public void initRobo() {
        topLeft = hardwareMap.dcMotor.get("topLeftMotor");
        topRight = hardwareMap.dcMotor.get("topRightMotor");
        bottomLeft = hardwareMap.dcMotor.get("bottomLeftMotor");
        bottomRight = hardwareMap.dcMotor.get("bottomRightMotor");
        intake = hardwareMap.dcMotor.get("intake");

        /*
        tSense = hardwareMap.get(RevColorSensorV3.class, "tSense");

        bSense = hardwareMap.get(RevColorSensorV3.class, "bSense");
         */

        launchLeft = hardwareMap.dcMotor.get("launchLeftMotor");
        launchRight = hardwareMap.dcMotor.get("launchRightMotor");
        /*
        clawGrab = hardwareMap.servo.get("grab");
        wrist = hardwareMap.crservo.get("wrist");

        rAP = hardwareMap.crservo.get("rAP");
         */
        fPin = hardwareMap.servo.get("fPin");

        /*
        Gyro = hardwareMap.get(BNO055IMU.class, "Gyro");
        Orientation orient = new Orientation();
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        Gyro.initialize(parameters);
        telemetry.addData("Calibrating Gyro: ", Gyro.getCalibrationStatus().toString());
        telemetry.addData("Gyro ready?: ", Gyro.isGyroCalibrated());
         */


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

        launchLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launchRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /*
        clawGrab.setDirection(Servo.Direction.FORWARD);
        wrist.setDirection(CRServo.Direction.FORWARD);
        rAP.setDirection(CRServo.Direction.FORWARD);
        fPin.setDirection(Servo.Direction.FORWARD);
*/
        launchLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        launchRight.setDirection(DcMotorSimple.Direction.FORWARD);

        topLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        topLeft.setPower(0);
        topRight.setPower(0);
        bottomLeft.setPower(0);
        bottomRight.setPower(0);

        launchLeft.setPower(0);
        launchRight.setPower(0);

        /*
        rAP.setPower(0);
        wrist.setPower(0);


         */
        launchLeft.setPower(0);
        launchRight.setPower(0);

        fPin.setPosition(0.5);
        /*
        clawGrab.setPosition(1);

        bSense.enableLed(true);
        tSense.enableLed(true);
        bSense.initialize();

        tSense.initialize();
        */

        /*
        topRight.setMode(bottomLeft.getMode());
        topLeft.setMode(bottomLeft.getMode());
        bottomRight.setMode(bottomLeft.getMode());
        launchLeft.setMode(launchRight.getMode());
         */
        //powerBase = 12 / this.hardwareMap.voltageSensor.iterator().next().getVoltage();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        waitForStart();
    }

    public void turnHeading(double angle) throws InterruptedException {
        double subAng = angle - 0.5;
        double supAng = angle + 0.5;
        while (opModeIsActive()) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("Currently buffering, position :", angles.firstAngle);
            telemetry.update();
            if (angle < angles.firstAngle) {
                topLeft.setPower(-0.05);
                topRight.setPower(0.05);
                bottomLeft.setPower(-0.05);
                bottomRight.setPower(0.05);
                if (subAng < angles.firstAngle && angles.firstAngle < supAng) {
                    break;
                }
            } else if (angle > angles.firstAngle) {
                topLeft.setPower(0.05);
                topRight.setPower(-0.05);
                bottomLeft.setPower(0.05);
                bottomRight.setPower(-0.05);
                if (subAng < angles.firstAngle && angles.firstAngle < supAng) {
                    break;
                }
            }
        }
        topLeft.setPower(0);
        topRight.setPower(0);
        bottomLeft.setPower(0);
        bottomRight.setPower(0);
    }

    public void setAllRun() {
        topLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        topRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottomLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottomRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setPowerAll(double pow) {
        topLeft.setPower(pow);
        topRight.setPower(pow);
        bottomLeft.setPower(pow);
        bottomRight.setPower(pow);
    }

    /*
        public void setPowerAll(double powTL, double powTR, double powBL, double powBR) {
            topLeft.setPower(powTL*powerBase);
            topRight.setPower(powTR*powerBase);
            bottomLeft.setPower(powBL*powerBase);
            bottomRight.setPower(powBR*powerBase);
        }
    */
    public void setPowerAll(double powTL, double powTR, double powBL, double powBR) {
        topLeft.setPower(powTL);
        topRight.setPower(powTR);
        bottomLeft.setPower(powBL);
        bottomRight.setPower(powBR);
    }


    public void setPosAll(int pos) {
        bottomLeft.setTargetPosition(pos);
    }

    public void setPosAll(int posTL, int posTR, int posBL, int posBR) {
        topLeft.setTargetPosition(posTL);
        topRight.setTargetPosition(-posTR);
        bottomLeft.setTargetPosition(posBL);
        bottomRight.setTargetPosition(-posBR);
    }

    public void encoderFront(int pos, double pow) throws InterruptedException {
        setPosAll(pos);

        setAllRun();

        setPowerAll(pow);

        while (topLeft.isBusy()) ;

        setPowerAll(0);
    }

    public void fRbR(int time, double pow) throws InterruptedException {
        setPowerAll(pow);

        Thread.sleep(time);

        setPowerAll(0);
    }

    public void sideways(int time, double pow) throws InterruptedException {

        setPowerAll(-pow, pow, pow, -pow);

        Thread.sleep(time);

        setPowerAll(0);
    }

    public void rotation(int time, double pow) throws InterruptedException {

        setPowerAll(pow, -pow, pow, -pow);

        Thread.sleep(time);

        setPowerAll(0);
    }

    public void diagonal(int time, double powLTRB, double powRTLB) throws InterruptedException {

        setPowerAll(powLTRB, powRTLB, powRTLB, powLTRB);

        Thread.sleep(time);

        setPowerAll(0);
    }
    /*
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
     */

    public void deployLauncher(double pow) throws InterruptedException {
        launchLeft.setPower(pow);
        launchRight.setPower(pow);
    }

    public void startIntake(double pow, double pos, int time) throws InterruptedException {
        intake.setPower(pow);
        fPin.setPosition(pos);

        Thread.sleep(time);
    }

    public void returnfPin() throws InterruptedException {
        fPin.setPosition(0.5);
    }

}

    /*
    public double getAngle(double angle){
        if(angle<0){
            return Math.abs(angle);
        }
        else if(angle>360){
            return Math.abs(angle) - 360;
        }
        else{
            return 360 - angle;
        }
    }
    public boolean rightorleft(int angle){
        if(Math.abs(Gyro.getAngularOrientation().firstAngle - angle)>180)
            return true;
        else
            return false;
    }
    public void turngyro(int angle){
        //double heading = imu.getAngularOrientation().firstAngle;
        if(Math.abs(getAngle(Gyro.getAngularOrientation().firstAngle) - angle) != 0){
            if(angle == 0){
                telemetry.addData("Current angle: ", Gyro.getAngularOrientation().firstAngle);
            }
            else if(rightorleft(angle)){
                topLeft.setPower(-0.35);
                topRight.setPower(0.35);
                bottomLeft.setPower(-0.35);
                bottomRight.setPower(0.35);
            }
            else{
                topLeft.setPower(0.35);
                topRight.setPower(-0.35);
                bottomLeft.setPower(0.35);
                bottomRight.setPower(-0.35);
            }
        }
     */