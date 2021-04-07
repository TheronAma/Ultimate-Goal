package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystem.wobblegoal.Wobble;
import org.firstinspires.ftc.teamcode.subsystem.wobblegoal.WobbleConstants;

@Disabled
@Config
@TeleOp()
public class WobbleClawTest extends LinearOpMode {

    public static double TARGET_CLAW_POS = 0.5;
    public static int TARGET_ARM_POS = WobbleConstants.TARGET_ARM_POS;

    @Override
    public void runOpMode() throws InterruptedException{
        Wobble wobble = new Wobble(hardwareMap);

        waitForStart();
        wobble.setTargetPosition(TARGET_ARM_POS);
        wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobble.setPower(1);

        while(opModeIsActive()){
            wobble.setTargetPosition(TARGET_ARM_POS);
            wobble.setClawPosition(TARGET_CLAW_POS);
        }
    }
}
