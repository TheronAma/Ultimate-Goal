package org.firstinspires.ftc.teamcode.subsystem.shooter;

import com.acmerobotics.dashboard.config.Config;

import java.util.ArrayList;
import java.util.List;

@Config
public class FlywheelLUT {

    private static ArrayList<ArrayList<Double>> dataPoints = new ArrayList<>(30); //2d array to store data points of the

    public static double calculate(double distance) {
        for (int i = 0; i < dataPoints.size() - 1; i++) {
            if (distance >= dataPoints.get(i).get(1)) {
                double slope = (dataPoints.get(i + 1).get(1) - dataPoints.get(i).get(1)) /
                        (dataPoints.get(i + 1).get(0) - dataPoints.get(i).get(0));
                return dataPoints.get(i).get(0) + (distance - dataPoints.get(i).get(1)) / slope;
            }
        }
        return dataPoints.get(29).get(1);
    }

}




