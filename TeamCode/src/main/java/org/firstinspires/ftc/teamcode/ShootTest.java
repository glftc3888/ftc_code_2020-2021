package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.Parent;
@Autonomous(name = "Shoot_Test", group = "autonomous")
public class ShootTest extends Parent{
    public void runOpMode() throws InterruptedException {
        initRobo();
        powerBase = 12/this.hardwareMap.voltageSensor.iterator().next().getVoltage();
        deployLauncher(1,2000);
        Thread.sleep(2000);
        deployLauncher(1,2000);
        Thread.sleep(2000);
        deployLauncher(1,2000);
        //YAY

    }
}
