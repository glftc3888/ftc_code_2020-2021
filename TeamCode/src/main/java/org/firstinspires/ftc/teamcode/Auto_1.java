package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Parent;

@Autonomous(group = "autonomous", name = "b")
public class Auto_1 extends Parent {
    public void runOpMode() throws InterruptedException {
            initRobo();
            waitForStart();
            fRbR(1000, .25);
            // sideways(-200,.25);

        }

}