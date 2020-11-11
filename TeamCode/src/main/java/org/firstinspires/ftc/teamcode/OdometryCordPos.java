package org.firstinspires.ftc.teamcode;

import android.graphics.drawable.GradientDrawable;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.sql.Time;
/*
public class OdometryCordPos extends Runnable {

    private DcMotor xAxis, yAxis;

    double xAxisPos = 0, yAxisPos = 0, changeInRobotOrientation = 0;


    double xAxisX, yAxisY;

    double prevX, prevY;


    private int xAxisPosMulti = 1, yAxisPosMulti = 1;

    private int sleepTime;

    private double globalAngle, power = .30;


    BNO055IMU gyro ;
    Orientation lastAngles = new Orientation();

    public OdometryCordPos(DcMotor x, DcMotor y, int time) throws InterruptedException{
        xAxis = x;
        yAxis = y;
        sleepTime = time;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;

        gyro.initialize(parameters);

        while (!gyro.isGyroCalibrated());
    }

    private void odomUpdate(){
        xAxisPos = xAxis.getCurrentPosition() * xAxisPosMulti;
        yAxisPos = yAxis.getCurrentPosition() * yAxisPosMulti;

        double xChange = xAxisPos - prevX;
        double yChange = yAxisPos - prevY;


    }


    private double getAngle(){
        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    private void resetAngle(){
        lastAngles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        globalAngle = 0;
    }


}
 */