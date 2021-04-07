package org.firstinspires.ftc.teamcode.subsystem.shooter;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Barrel {

    Servo arm;
    Servo pusher;

    boolean isRaised;
    boolean isExtended;

    public Barrel(HardwareMap hardwareMap){
        arm = hardwareMap.get(Servo.class,"barrel");
        pusher = hardwareMap.get(Servo.class, "pusher");

        isRaised = false;
        isExtended = false;
    }

    public void raise(){ arm.setPosition(Const.BARREL_TOP_POS); isRaised = true; retract(); }

    public void lower(){ arm.setPosition(Const.BARREL_BOTTOM_POS); isRaised = false; }

    public void extend(){
        if(isRaised){
            pusher.setPosition(Const.PUSHER_EXTEND_POS);
            isExtended = true;
        }
    }

    public void retract(){ pusher.setPosition(Const.PUSHER_RETRACT_POS); isExtended = false; }

    public void setArmPosition(double position){
        arm.setPosition(position);
    }

    public void setPusherPosition(double position){ pusher.setPosition(position); }

    public void toggleArm() {
        if (isRaised) {
            lower();
        } else {
            raise();
        }
    }

    public void togglePusher(){
        if(isExtended){
            retract();
        } else {
            extend();
        }
    }



}
