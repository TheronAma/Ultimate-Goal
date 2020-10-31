package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.opmode.OpModeBase;
import org.firstinspires.ftc.teamcode.subsystem.shooter.Shooter;

/*
 * Diagnostic opmode in order to test functionality of shooter methods
 * Will run the flywheel slowly and measure velocity over time
 */
@Config
public class ShooterTest extends OpModeBase {
    public static double TARGET_RPM = 100;
    public void runOpMode(){

        Shooter shooter = new Shooter(hardwareMap);

        waitForStart();

        shooter.setVelocity(TARGET_RPM);
        while(opModeIsActive()){
            shooter.update();
            telemetry.addData("Rpm",shooter.getVelocity());
        }
    }
}
