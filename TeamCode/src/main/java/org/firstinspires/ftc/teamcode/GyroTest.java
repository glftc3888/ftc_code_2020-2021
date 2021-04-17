package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Parent;
/*

@Autonomous(name = "testing", group = "autonomous")
 */
public class GyroTest extends Parent {


    public void runOpMode() throws InterruptedException {
        initRobo();
        fRbR(1100,-.15);
        Thread.sleep(1000); //wait
        sideways(1250,-.1);
        Thread.sleep(1000);
        turnHeading(0);
        Thread.sleep(1000);
        telemetry.addLine("Auto 1");
        telemetry.update();
        turnHeading(90);
        Thread.sleep(1000); //wait
        turnHeading(179);
        Thread.sleep(1000); //wait
        fRbR(1400,0.15);
        Thread.sleep(1000);
        sideways(100,.1);
        Thread.sleep(1000);
        sideways(500,.5);
    }

}