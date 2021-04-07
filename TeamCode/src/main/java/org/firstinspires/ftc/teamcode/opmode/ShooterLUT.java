package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.util.InterpLUT;

import java.util.Arrays;
import java.util.List;

public class ShooterLUT {

    public List<Double> input = Arrays.asList(62.92, 67.3, 70.85, 74.0, 77.1,80.1, 83.98, 87.77,90.11, 93.69, 97.1, 100.2, 103.1, 106.5, 110.0);
    public List<Double> output = Arrays.asList(39.0, 37.0, 33.0, 31.5, 30.0, 29.0, 26.0, 27.0, 25.0, 23.0, 21.0, 19.0, 17.0, 15.0, 15.0);

    public InterpLUT lookUp = new InterpLUT();

    public  ShooterLUT(){
        for(int i = 0; i < input.size(); i++){
            lookUp.add(input.get(i),output.get(i));
        }
        lookUp.createLUT();
    }

    public double get(double a){
        return lookUp.get(a);
    }
}
