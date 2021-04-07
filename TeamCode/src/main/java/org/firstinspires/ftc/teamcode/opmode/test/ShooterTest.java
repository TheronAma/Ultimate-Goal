package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmode.OpModeBase;
import org.firstinspires.ftc.teamcode.subsystem.shooter.Barrel;
import org.firstinspires.ftc.teamcode.subsystem.shooter.Shooter;

/*
 * Diagnostic opmode in order to test functionality of shooter methods
 * Will run the flywheel slowly and measure velocity over time
 */
//@Disabled
@Config
@TeleOp()
public class ShooterTest extends OpModeBase {
    public static double TARGET_RPM = 4000;
    public static double TARGET_ANGLE = 20; //a number 0 - 100, where 0 is the flat, and 100 is the maximum angle

    private boolean aWasPressed = false;
    private boolean bWasPressed = false;



    public void runOpMode(){
        Shooter shooter = new Shooter(hardwareMap);
        Barrel barrel = new Barrel(hardwareMap);
        waitForStart();
        while(opModeIsActive()){
            shooter.setRpm(TARGET_RPM);
            shooter.setAngle(TARGET_ANGLE);
            shooter.sendDashTelemetry();

            if(gamepad1.a && !aWasPressed){
                barrel.toggleArm();
                aWasPressed = true;
            } else if(!gamepad1.b){
                aWasPressed = false;
            }

            if(gamepad1.b && !bWasPressed){
                barrel.togglePusher();
                bWasPressed = true;
            } else if(!gamepad1.b){
                bWasPressed = false;
            }

        }
    }
}
