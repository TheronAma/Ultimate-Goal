package org.firstinspires.ftc.teamcode.subsystem.shooter;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Barrel {

    Servo arm;
    Servo pusher;

    public Barrel(HardwareMap hardwareMap){
        arm = hardwareMap.get(Servo.class,"barrel");
        pusher = hardwareMap.get(Servo.class, "pusher");
    }

    public void raise(){ arm.setPosition(Const.BARREL_TOP_POS); }

    public void lower(){
        arm.setPosition(Const.BARREL_BOTTOM_POS);
    }

    public void extend(){
        pusher.setPosition(Const.PUSHER_EXTEND_POS);
    }

    public void retract(){
        pusher.setPosition(Const.PUSHER_RETRACT_POS);
    }

    public void setArmPosition(double position){
        arm.setPosition(position);
    }

    public void setPusherPosition(double position){ pusher.setPosition(position); }

}
