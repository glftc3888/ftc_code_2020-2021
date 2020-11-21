package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "third one", group = "autonomous")
public class Auto_3 extends Parent{

    public void runOpMode() throws InterruptedException {
        initRobo();
        fRbR(2400, -.125);
        fRbR(500, 0);
        rotation(1500, -.0625);
        moveOpen(2000, -1);
        clawMove(1000,1);
        fRbR(900, -.125);
    }

}
