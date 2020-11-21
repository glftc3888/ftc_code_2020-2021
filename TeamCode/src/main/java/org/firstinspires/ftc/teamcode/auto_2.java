package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto_2", group = "autonomous")
public class auto_2 extends Parent{

    public void runOpMode() throws InterruptedException {
        initRobo();
        fRbR(2100, -.125);
        fRbR(500, 0);
        moveGrip(0);
        fRbR(800,.125);
    }
}
