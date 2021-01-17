package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.opmode.OpModeBase;
import org.firstinspires.ftc.teamcode.subsystem.shooter.Barrel;
import org.firstinspires.ftc.teamcode.subsystem.shooter.Shooter;

@Config
public class PusherTest extends OpModeBase {
    public static double PUSHER_POSITION = 0.2;
    public void runOpMode(){

        Barrel barrel = new Barrel(hardwareMap);

        waitForStart();


        while(opModeIsActive()){
            barrel.setPusherPosition(PUSHER_POSITION);

            telemetry.addData("Pusher Position", PUSHER_POSITION);
            telemetry.update();
        }
    }
}