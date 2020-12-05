package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto_3", group = "autonomous")
public class Auto_3 extends Parent{

    public void runOpMode() throws InterruptedException {
        initRobo();
        fRbR(2400, -.125);
        rotation(1500, .0625);
        moveGrip(0);
        fRbR(900, -.125);
    }

}
