package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "second one", group = "autonomous")
public class auto_2 extends Parent{

    public void runOpMode() throws InterruptedException {
        initRobo();
        waitForStart();
        fRbR(1500, .25);
    }
}
