package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmode.OpModeBase;
import org.firstinspires.ftc.teamcode.subsystem.shooter.Shooter;

/*
 * Diagnostic opmode in order to test functionality of shooter methods
 * Will run the flywheel slowly and measure velocity over time
 */
@Config
@TeleOp()
public class ShooterTest extends OpModeBase {
    public static double TARGET_VELOCITY = 4000;
    public void runOpMode(){
        Shooter shooter = new Shooter(hardwareMap);
        waitForStart();
        while(opModeIsActive()){
            shooter.setPower(1);
            telemetry.addData("Shooter velocity",shooter.getVelocity());
            telemetry.update();
            shooter.sendDashTelemetry();
        }
    }
}
