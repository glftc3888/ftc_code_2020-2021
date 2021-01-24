package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.Parent;

@Autonomous(name = "GyroTest", group = "autonomous")
public class GyroTest extends Parent {

    public void runOpMode() throws InterruptedException {
        initRobo();
        turnHeading(90);


    }

}