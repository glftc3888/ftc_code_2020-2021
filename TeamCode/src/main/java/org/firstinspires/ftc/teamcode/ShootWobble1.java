package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.Parent;

@Autonomous(name = "Shoot_Wobble_1", group = "autonomous")
public class ShootWobble1 extends Parent {

    public void runOpMode() throws InterruptedException {
        initRobo();
        //Align and Shoot
        fRbR(500,.05);
        Thread.sleep(500);
        sideways(500,-1);
        deployLauncher(0.962,1000);
        Thread.sleep(1000);
        deployLauncher(0.962,1000);
        Thread.sleep(1000);
        deployLauncher(0.962,1000);
        Thread.sleep(1000);
        fRbR(1875, .05);
        fRbR(1550,.025);
        fRbR(1000, 0);
        sideways(500,1);
        rotation(1125,-.125);
        // Moves arm and claw
        moveRacPin(2500, -1);
        moveWrist(1000, -1);
        moveGrip(0);
    }

}