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
            rotation(1500, -.0625);
            moveOpen(3500, -1);
        }

}