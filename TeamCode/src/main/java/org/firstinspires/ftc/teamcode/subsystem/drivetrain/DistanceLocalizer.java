package org.firstinspires.ftc.teamcode.subsystem.drivetrain;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class DistanceLocalizer {

    private DistanceSensor backSensor, leftSensor, rightSensor;

    private Pose2d poseEstimate = new Pose2d();

    public DistanceLocalizer(HardwareMap hardwareMap) {
        backSensor = new DistanceSensor(hardwareMap,"backDistance");
        leftSensor = new DistanceSensor(hardwareMap,"leftDistance");
        rightSensor = new DistanceSensor(hardwareMap,"rightDistance");
    }

    public List<Double> getDistances(){
        return Arrays.asList(leftSensor.getDistance(), backSensor.getDistance(), rightSensor.getDistance());
    }

}
