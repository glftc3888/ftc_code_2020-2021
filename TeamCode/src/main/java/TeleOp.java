import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "TeleOp")

public class TeleOp extends OpMode {
    ElapsedTime a = new ElapsedTime();
    DcMotor topLeft;
    DcMotor topRight;
    DcMotor bottomLeft;
    DcMotor bottomRight;
    DcMotor intake;
    DcMotor launchLeft;
    DcMotor launchRight;
    //DcMotor reinforceLaunch;


    Servo clawGrab;
    Servo fPin;
    CRServo rAP;
    CRServo wrist;

    //RevColorSensorV3 tSense;
    //RevColorSensorV3 bSense;


    double powerBase;
    boolean on_off = false;
    int pinPower;
    int intakePower;

    public void init(){
        topLeft = hardwareMap.dcMotor.get("topLeftMotor");
        topRight = hardwareMap.dcMotor.get("topRightMotor");
        bottomLeft = hardwareMap.dcMotor.get("bottomLeftMotor");
        bottomRight = hardwareMap.dcMotor.get("bottomRightMotor");
        intake = hardwareMap.dcMotor.get("intake");
        launchLeft = hardwareMap.dcMotor.get("launchLeftMotor");
        launchRight = hardwareMap.dcMotor.get("launchRightMotor");


        //tSense = hardwareMap.get(RevColorSensorV3.class, "tSense");
        //bSense = hardwareMap.get(RevColorSensorV3.class, "bSense");


        clawGrab = hardwareMap.servo.get("grab");
        wrist = hardwareMap.crservo.get("wrist");
        rAP = hardwareMap.crservo.get("rAP");
        fPin = hardwareMap.servo.get("fPin");


        topLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launchLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launchRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        topLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        topRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bottomLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        bottomRight.setDirection(DcMotorSimple.Direction.FORWARD);




        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        launchLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        launchRight.setDirection(DcMotorSimple.Direction.FORWARD);

        clawGrab.setDirection(Servo.Direction.FORWARD);
        wrist.setDirection(CRServo.Direction.FORWARD);
        rAP.setDirection(CRServo.Direction.FORWARD);
        fPin.setDirection(Servo.Direction.FORWARD);
        /*
        topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        topLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        */
        telemetry.addData("top left power", topLeft.getPower());
        telemetry.addData("top right power", topRight.getPower());
        telemetry.addData("bottom left power", bottomLeft.getPower());
        telemetry.addData("bottom right power", bottomRight.getPower());
        //telemetry.addData("tSense red: ", tSense.red());
        //telemetry.addData("bSense red: ", bSense.red());


        topLeft.setPower(0);
        topRight.setPower(0);
        bottomLeft.setPower(0);
        bottomRight.setPower(0);

        intake.setPower(0);
        launchLeft.setPower(0);
        launchRight.setPower(0);

        rAP.setPower(0);
        wrist.setPower(0);
        fPin.setPosition(0.5);

        topRight.setMode(bottomLeft.getMode());
        topLeft.setMode(bottomLeft.getMode());
        bottomRight.setMode(bottomLeft.getMode());

        launchLeft.setMode(launchRight.getMode());
    }


    public void loop() {
        powerBase = 12/this.hardwareMap.voltageSensor.iterator().next().getVoltage();
        double inmax = 0.75;
        /*
        telemetry.addData("top left encoder", topLeft.getCurrentPosition());
        telemetry.addData("top right encoder", topRight.getCurrentPosition());
        telemetry.addData("bottom left encoder", bottomLeft.getCurrentPosition());
        telemetry.addData("bottom right encoder", bottomRight.getCurrentPosition());
        */

        //Turbo
        if(gamepad1.right_trigger == 1) {
            topLeft.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x);
            topRight.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x);
            bottomLeft.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x);
            bottomRight.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x);
        }

        //Normal Movement

        topLeft.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x) * .5);
        topRight.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) * .5);
        bottomLeft.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x) * .5);
        bottomRight.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x) * .5);





        rAP.setPower(-gamepad2.left_stick_y);

        //Claw Movement
        if (gamepad2.a) {
            clawGrab.setPosition(0);
        }
        if(gamepad2.b) {
            clawGrab.setPosition(1);
        }

        //Servo for rotating wobble goal
        wrist.setPower(gamepad2.right_stick_x);

        //Launch and intake motors
        telemetry.addData("PewPew Left", launchLeft.getPower());
        telemetry.addData("PewPew Right", launchRight.getPower());



        launchLeft.setPower(gamepad2.right_trigger*powerBase);
        launchRight.setPower(gamepad2.right_trigger*powerBase);

        intake.setPower(gamepad1.left_trigger);
        if(intake.getPower()>inmax)
            intake.setPower(inmax);
        if (gamepad1.dpad_down)
            intake.setPower(-1);

        //Servo that moves the disc into the launcher
        if(gamepad2.right_bumper)
            fPin.setPosition(-0.5);
        else
            fPin.setPosition(0.5);

        telemetry.addData("Chacha Power",this.hardwareMap.voltageSensor.iterator().next().getVoltage());
        telemetry.addData("ChaCha Power Multiplier", powerBase);

        /*
        Color Sensor Range normal: 100 to 300
        Color Sensor Range with ring: 4000 to 5000
        auto1 = bSense false
        auto2 = bSense true, tSense false
        auto3 = bSense true, tSense true
         */
    }

}
