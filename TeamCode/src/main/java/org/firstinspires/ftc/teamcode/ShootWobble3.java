package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.Parent;

@Autonomous(name = "Shoot_Wobble_3", group = "autonomous")
public class ShootWobble3 extends Parent {

    public void runOpMode() throws InterruptedException {
        initRobo();
        //Align and Shoot
        fRbR(500, .05);
        Thread.sleep(500);
        sideways(1300, -0.1);
        Thread.sleep(1000);
        deployLauncher(0.96, 1000);
        Thread.sleep(1000);
        deployLauncher(0.96, 1000);
        Thread.sleep(1000);
        deployLauncher(0.96, 1000);
        //Align for WobbleGoal placement
        Thread.sleep(1000);
        sideways(650,0.25);
        Thread.sleep(1000);
        fRbR(3650, 0.05);
        fRbR(1000, 0);

        rotation(1150,-0.1);
        // Moves arm and claw
        moveRacPin(2750, -1);
        moveWrist(1000, -1);
        moveGrip(0);
        moveWrist(1000,1);
        moveRacPin(2750,1);
        moveGrip(1);
        sideways(1150,1);
    }

}