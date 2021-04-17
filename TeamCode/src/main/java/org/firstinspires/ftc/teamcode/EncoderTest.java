package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.Parent;

@Autonomous(name = "EncoderTest", group = "autonomous")
public class EncoderTest extends Parent2 {

    public void runOpMode() throws InterruptedException {
        initRobo();
        moveSideways(4389,1);
        move(30723,1);
        turn(4000,1);
        moveSideways(8778,1);
    }
}