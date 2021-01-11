package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Wobble_2", group = "autonomous")
public class auto_2 extends Parent{

    public void runOpMode() throws InterruptedException {
        initRobo();
        // Moves bot to second box
        fRbR(3250, -.05);
        fRbR(300,-.025);
        fRbR(1000, 0);
        // Rotates bot into position
        rotation(1125, .125);
        rotation(1000, 0);
        // Moves arm and claw
        moveRacPin(2600, -1);
        moveWrist(1000, -1);
        moveWrist(1000, 0);
        moveGrip(0);
        // Moves bot towards launch line
        fRbR(2000, -.05);
        fRbR(1000, 0);
    }
}
