package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//Primary Time Autonomous (holds all three events)
@Autonomous(name = "time_Triad", group = "autononmous")
public class timeTriad extends Parent{
    public void runOpMode() throws InterruptedException {
        initRobo();
        powerBase = 12/this.hardwareMap.voltageSensor.iterator().next().getVoltage();
        fRbR(600,0.5);
        Thread.sleep(1000); //wait
        rotation(500,.125);
        Thread.sleep(1000);
        fRbR(500,0.5);
        Thread.sleep(1000);
        rotation(500,-0.125);
        Thread.sleep(1000);
        if(260 >= bSense.red()){
            //auto1
            rotation(800,.25);
            Thread.sleep(1000); //wait
            fRbR(1000,-0.5);
            Thread.sleep(1000);
            moveRacPin(2500, -1);
            moveWrist(1000, -1);
            moveGrip(0);

        }
        else if(260 <= bSense.red() && 4000 >= tSense.red()){
            //auto2
            fRbR(1000,0.5);
            Thread.sleep(1000); //wait
            sideways(800,-0.12);
            moveRacPin(2500, -1);
            moveWrist(1000, -1);
            moveGrip(0);
            fRbR(800,-0.5);
        }
        else if(260 <= bSense.red() && 4000 <= tSense.red()){
            //auto3
            rotation(800,.25);
            Thread.sleep(1000); //wait
            fRbR(2000, -0.5);
            moveRacPin(2500, -1);
            moveWrist(1000, -1);
            moveGrip(0);
            fRbR(1600,-0.5);
        }
    }
}