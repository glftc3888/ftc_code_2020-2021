package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//Primary Autonomous (holds all three events)
/*

@Autonomous(name = "autoTriad", group = "autononmous")

public class autoTriad extends Parent2{
    public void runOpMode() throws InterruptedException {
        initRobo();
        powerBase = 12/this.hardwareMap.voltageSensor.iterator().next().getVoltage();
        move(.50, 2400);
        Thread.sleep(1500); //wait
        moveSideways(.50,1400);
        Thread.sleep(1000);
        if(190 > bSense.red()){
            //auto1
            turn(.50, 1400);
            Thread.sleep(1000); //wait
            move(.50, -105);
            moveRacPin(2500, -1);
            moveWrist(1000, -1);
            moveGrip(0);

        }
        else if(190 <= bSense.red() && 4000 >= tSense.red()){
            //auto2
            move(.50,210);
            Thread.sleep(1000); //wait
            moveSideways(.50,-70);
            moveRacPin(2500, -1);
            moveWrist(1000, -1);
            moveGrip(0);
            move(.50,-210);
        }
        else if(260 <= bSense.red() && 4000 <= tSense.red()){
            //auto3
            turn(.50, 800);
            Thread.sleep(1000); //wait
            move(.50, -245);
            moveRacPin(2500, -1);
            moveWrist(1000, -1);
            moveGrip(0);
            move(.50,245);
        }
    }

}
 */