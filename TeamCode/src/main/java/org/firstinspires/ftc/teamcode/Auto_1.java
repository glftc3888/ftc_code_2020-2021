package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.Parent;

@Autonomous(name = "Wobble_1", group = "autonomous")
public class Auto_1 extends Parent {

    public void runOpMode() throws InterruptedException {
            initRobo();
            // Move bot to the first box
            fRbR(1750, -.05);
            fRbR(1550,-.025);
            fRbR(1000, 0);
            // Moves arm and claw
            moveRacPin(2500, -1);
            moveWrist(1000, -1);
            moveGrip(0);
    }

}