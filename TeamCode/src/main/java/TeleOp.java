import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOpV2", group = "TeleOp")

abstract class TeleOp extends OpMode {
    DcMotor topLeft = hardwareMap.dcMotor.get("Top left motor");
    DcMotor topRight = hardwareMap.dcMotor.get("Top right motor");
    DcMotor bottomLeft = hardwareMap.dcMotor.get("Bottom left motor");
    DcMotor bottomRight = hardwareMap.dcMotor.get("Bottom right motor");

    
}
