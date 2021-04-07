package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.shooter.Barrel;
import org.firstinspires.ftc.teamcode.subsystem.shooter.Shooter;

@Disabled
@Config
@TeleOp()
public class FlapTest extends LinearOpMode {
    public static double FLAP_POSITION = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        Shooter shooter = new Shooter(hardwareMap);
        waitForStart();
        while(!isStopRequested() && isStarted()){

            shooter.setPosition(FLAP_POSITION);
            telemetry.addLine("Current Barrel Flap Position: " + FLAP_POSITION);
            telemetry.update();
        }
    }
}
