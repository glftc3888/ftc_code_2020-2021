package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.Parent;

@Autonomous(name = "Auto_swag1", group = "autonomous")
public class Auto_1 extends Parent {

    public void runOpMode() throws InterruptedException {
            initRobo();
            fRbR(1700, -.125);
            fRbR(500, 0);
            rotation(1500, -.0625);
            moveGrip(0);
        }

}