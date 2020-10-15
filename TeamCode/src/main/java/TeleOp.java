import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOpV1", group = "TeleOp")

abstract class TeleOp extends OpMode {
    DcMotor topLeft;
    DcMotor topRight;
    DcMotor bottomLeft;
    DcMotor bottomRight;

    DcMotor intake;
    Servo servoLauncher;




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

        topLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        topRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        topLeft.setPower(0);
        topRight.setPower(0);
        bottomLeft.setPower(0);
        bottomRight.setPower(0);
    }

    public void loop(){
        topLeft.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x);
        topRight.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x);
        bottomLeft.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x);
        bottomRight.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x);
    }
}
