package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "second one", group = "autonomous")
public class auto_2 extends Parent{

    public void runOpMode() throws InterruptedException {
        initRobo();
        waitForStart();
        fRbR(2100, -.125);
        fRbR(500, 0);
        moveOpen(5000, -1);
        fRbR(800,.125);
    }
}
