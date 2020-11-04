package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(group = "autonomous", name = "big mans")
public class Auto_1 extends Parent {
    public void runOpMode() throws InterruptedException {
            initRobo();
            waitForStart();
            fRbR(570, .25);
            sideways(170,.25);

        }

}