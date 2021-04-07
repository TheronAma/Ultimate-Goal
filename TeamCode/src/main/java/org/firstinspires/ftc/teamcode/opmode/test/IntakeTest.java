package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.intake.Intake;

@Disabled
@Config
@TeleOp()
public class IntakeTest extends LinearOpMode {

    private Intake intake;
    public static double TARGET_POWER = 0.8;

    @Override
    public void runOpMode() throws InterruptedException {

        intake = new Intake(hardwareMap);

        waitForStart();
        while(opModeIsActive()){
            intake.setPower(TARGET_POWER);
        }
    }

}
