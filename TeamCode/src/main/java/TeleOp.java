import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "TeleOp")

public class TeleOp extends OpMode {
    ElapsedTime a = new ElapsedTime();
    DcMotor topLeft;
    DcMotor topRight;
    DcMotor bottomLeft;
    DcMotor bottomRight;
    DcMotor intake;
    DcMotor launch;
    //DcMotor reinforceLaunch;

    Servo clawGrab;
    Servo fPin;
    CRServo rAP; /* the */
    CRServo wrist;


    boolean on_off = false;
    int pinPower;
    int intakePower;

    public void init(){
        topLeft = hardwareMap.dcMotor.get("topLeftMotor");
        topRight = hardwareMap.dcMotor.get("topRightMotor");
        bottomLeft = hardwareMap.dcMotor.get("bottomLeftMotor");
        bottomRight = hardwareMap.dcMotor.get("bottomRightMotor");
        intake = hardwareMap.dcMotor.get("intake");
        launch = hardwareMap.dcMotor.get("launch");


        clawGrab = hardwareMap.servo.get("grab");
        wrist = hardwareMap.crservo.get("wrist");
        rAP = hardwareMap.crservo.get("rAP");
        fPin = hardwareMap.servo.get("fPin");


        topLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        topLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        topRight.setDirection(DcMotorSimple.Direction.REVERSE);
        bottomLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        bottomRight.setDirection(DcMotorSimple.Direction.REVERSE);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        launch.setDirection(DcMotorSimple.Direction.FORWARD);
        //reinforceLaunch.setDirection(DcMotorSimple.Direction.REVERSE);

        clawGrab.setDirection(Servo.Direction.FORWARD);
        wrist.setDirection(CRServo.Direction.FORWARD);
        rAP.setDirection(CRServo.Direction.FORWARD);
        fPin.setDirection(Servo.Direction.FORWARD);

        topLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        topLeft.setPower(0);
        topRight.setPower(0);
        bottomLeft.setPower(0);
        bottomRight.setPower(0);
        intake.setPower(0);
        launch.setPower(0);


        rAP.setPower(0);
        wrist.setPower(0);
    }


    public void loop() {
        telemetry.addData("top left ", topLeft.getCurrentPosition());
        telemetry.addData("top right ", topRight.getCurrentPosition());

        //Turbo
        if(gamepad1.right_trigger == 1) {
            topLeft.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x);
            telemetry.addData("TLpower ", topLeft.getPower());
            topRight.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x);
            telemetry.addData("TRpower ", topRight.getPower());
            bottomLeft.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x);
            telemetry.addData("BLpower ", bottomLeft.getPower());
            bottomRight.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x);
            telemetry.addData("BRpower ", bottomRight.getPower());
        }

        //Normal Movement
        topLeft.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x) * .25);
        topRight.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x) * .25);
        bottomLeft.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) * .25);
        bottomRight.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x) * .25);

        //Rack and pinion movement
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
        launch.setPower(gamepad2.right_trigger);
        intake.setPower(gamepad1.left_trigger);
        if (gamepad1.dpad_down)
            intake.setPower(-1);

        //Servo that moves the disc into the launcher
        pinPower = gamepad2.right_bumper ? 1 : 0;
        fPin.setPosition(pinPower);

    }

}
