package org.firstinspires.ftc.teamcode.opmode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous()
public class FlyWheelCharacterization extends LinearOpMode {

    private DcMotor flywheel;

    //Constants for the tuning
    public static double voltsPerSecond = 0.25;
    public static double accelWindow = 8;


    @Override
    public void runOpMode() {
        flywheel = hardwareMap.get(DcMotor.class,"flywheel");



    }

}
