package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "third one", group = "autonomous")
public class Auto_3 extends Parent{

    public void runOpMode() throws InterruptedException {
        initRobo();
        waitForStart();
        fRbR(1800, .25);
    }

}
