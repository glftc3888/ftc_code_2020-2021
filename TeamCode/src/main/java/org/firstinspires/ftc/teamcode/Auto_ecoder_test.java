package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.Parent;

@Autonomous(name = "--> Shooting Auto <--", group = "autonomous")
public class  Auto_ecoder_test extends Parent{
    public void runOpMode() throws InterruptedException {
        initRobo();
        telemetry.addLine("Ignore the filename. This is for a shooting autonomous. :)");
        telemetry.update();
        fRbR(500,-0.1);
        Thread.sleep(1000); //wait
        sideways(1300,-0.1);
        Thread.sleep(1000); //wait
        turnHeading(0);
        Thread.sleep(1000); //wait
        fRbR(1200,-0.1);
        fRbR(870,-0.001);
        Thread.sleep(1000); //wait
        turnHeading(6);
        Thread.sleep(1000); //wait
        deployLauncher(0.99); //>launcher starts<
        fPin.setPosition(-0.5); //<fire ring>
        Thread.sleep(500);
        returnfPin();
        Thread.sleep(1000); //wait
        turnHeading(2);
        Thread.sleep(1000); //wait
         //>launcher starts<
        fPin.setPosition(-0.5); //<fire ring>
        Thread.sleep(500);
        returnfPin();
        Thread.sleep(1000); //wait
        turnHeading(-4);
        Thread.sleep(1000); //wait
         //>launcher starts<
        fPin.setPosition(-0.5); //<fire ring>
        Thread.sleep(500);
        returnfPin();
        Thread.sleep(1000); //wait
        deployLauncher(0);
        Thread.sleep(1000); //wait
        fRbR(500,-.15);
    }
}