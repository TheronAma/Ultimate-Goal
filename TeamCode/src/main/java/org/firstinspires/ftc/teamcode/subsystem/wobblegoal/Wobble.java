package org.firstinspires.ftc.teamcode.subsystem.wobblegoal;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Wobble {
    private int startPos;

    private DcMotor arm;
    private Servo claw;


    private boolean isExtended = false;
    private boolean isOpen = false;

    public Wobble(HardwareMap hardwareMap){
        arm = hardwareMap.get(DcMotor.class,"wobble");
        claw = hardwareMap.get(Servo.class,"claw");

        startPos = 0;


    }

    public void open() { claw.setPosition(WobbleConstants.CLAW_OPEN_POSITION); isOpen = true; }

    public void close() { claw.setPosition(WobbleConstants.CLAW_CLOSE_POSITION); isOpen = false; }

    public void setClawPosition(double position) { claw.setPosition(position);}

    public void setPower(double power){
        arm.setPower(power);
    }

    public void setMode(DcMotor.RunMode mode){
        arm.setMode(mode);
    }

    public void extend() { arm.setTargetPosition(startPos + WobbleConstants.TARGET_ARM_POS); setMode(DcMotor.RunMode.RUN_TO_POSITION); setPower(0.8); isExtended = true; }

    public void retract() { arm.setTargetPosition(startPos); setMode(DcMotor.RunMode.RUN_TO_POSITION); setPower(0.8); isExtended = false; }

    public void setTargetPosition(int position){ arm.setTargetPosition(position); }

    public void toggleExtend() {
        if(isExtended){
            retract();
        } else {
            extend();
        }
    }

    public void toggleOpen() {
        if(isOpen){
            close();
        } else {
            open();
        }
    }

}
