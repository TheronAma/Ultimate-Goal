package org.firstinspires.ftc.teamcode.subsystem.drivetrain;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DistanceSensor{

    AnalogInput sensor;

    public static double INCHES_PER_VOLT = 3.3/512;

    public DistanceSensor(HardwareMap hardwareMap, String name){
        sensor = hardwareMap.get(AnalogInput.class,name);
    }

    //returns distance in inches
    public double getDistance(){
        return sensor.getVoltage() * INCHES_PER_VOLT;
    }
}
