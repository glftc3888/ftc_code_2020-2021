package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Wobble_3", group = "autonomous")
public class Auto_3 extends Parent{

    public void runOpMode() throws InterruptedException {
        initRobo();
        // Moves bot to third box
        fRbR(3450, -.05);
        fRbR(450,-.025);
        // Moves arm and claw
        //moveRacPin(2650, -1);
        //moveWrist(1000, -1);
        //moveWrist(1000, 0);
        moveGrip(0);
        //moveRacPin(2650,1);
        moveGrip(1);
        // Moves bot to launch line
        fRbR(2050, .05);
    }

}
