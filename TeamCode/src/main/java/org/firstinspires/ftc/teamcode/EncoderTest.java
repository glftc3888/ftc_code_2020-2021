package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.Parent;

@Autonomous(name = "EncoderTest", group = "autonomous")
public class EncoderTest extends Parent2 {

    public void runOpMode() throws InterruptedException {
        initRobo();
        move(0.8,1500);
        moveSideways(0.8,1500);
        move(-0.8,-1500);
        moveSideways(-0.8,-1500);
        turn(1, 2000);
        turn(0.5,500);
        wait(2000);
        turn(-1,-2000);
        turn(-0.5,-500);
    }

}