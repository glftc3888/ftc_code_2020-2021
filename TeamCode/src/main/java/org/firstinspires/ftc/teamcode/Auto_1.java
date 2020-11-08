package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;

@Autonomous(group = "autonomous", name = "b")
public class Auto_1 extends Parent {
    public void runOpMode() throws InterruptedException {
            initRobo();
            waitForStart();
            fRbR(1700, -.125);
            fRbR(500, 0);
            rotation(1300, -.0625);
            rotation(550, -.05);
            rotation(1000,0);
            diagonal(1000, -.125, -.125);
            // sideways(-200,.25);

        }

}