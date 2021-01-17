package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystem.shooter.Barrel;

@Config
public class BarrelTest extends LinearOpMode {

    public static double ARM_POSITION = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        Barrel barrel = new Barrel(hardwareMap);
        waitForStart();
        while(!isStopRequested() && isStarted()){
            barrel.setArmPosition(ARM_POSITION);
            telemetry.addLine("Current Barrel Arm Position: " + ARM_POSITION);
            telemetry.update();
        }
    }
}
