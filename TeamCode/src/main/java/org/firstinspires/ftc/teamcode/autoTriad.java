package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//Primary Autonomous (holds all three events)
@Autonomous(name = "auto_Triad", group = "autononmous")
public class autoTriad extends Parent2{
    public void runOpMode() throws InterruptedException {
        initRobo();
        powerBase = 12/this.hardwareMap.voltageSensor.iterator().next().getVoltage();
        move(0.125, 70);
        moveSideways(0.125,70);
        if(4000 >= bSense.red()){
            //auto1
            turn(.125, 800);
            move(.125, -105);
            moveRacPin(2500, -1);
            moveWrist(1000, -1);
            moveGrip(0);

        }
        else if(4000 <= bSense.red() && 4000 >= tSense.red()){
            //auto2
            move(.125,210);
            moveSideways(.125,-70);
            moveRacPin(2500, -1);
            moveWrist(1000, -1);
            moveGrip(0);
            move(.125,-210);
        }
        else if(4000 <= bSense.red() && 4000 <= tSense.red()){
            //auto3
            turn(.125, 800);
            move(.125, -245);
            moveRacPin(2500, -1);
            moveWrist(1000, -1);
            moveGrip(0);
            move(.125,245);
        }
    }
}
