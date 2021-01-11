package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "encoder auto", group = "autonomous")
public class Auto_ecoder_test extends Parent{

    public void runOpMode() throws InterruptedException {
        encoderFront(1000, .25);
        encoderFront(0, 0);
    }
}
