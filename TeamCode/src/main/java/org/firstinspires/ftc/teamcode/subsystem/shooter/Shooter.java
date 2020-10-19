package org.firstinspires.ftc.teamcode.subsystem.shooter;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Shooter {
        DcMotorEx flywheel;
        Servo flap;

        public static double kV = 0.01;
        PIDCoefficients velocityPID = new PIDCoefficients(1,0,0);

        PIDFController velocityController;

        public Shooter(HardwareMap hwMap){
                flywheel = hwMap.get(DcMotorEx.class,"flywheel");
                flap = hwMap.get(Servo.class,"flap");
                velocityController = new PIDFController(velocityPID,kV);

        }

        public void setPower(double power){
                flywheel.setPower(power);
        }

        public void setVelocity(double rpm){

        }

        public void setAngle(double angle){

        }

        public void update(){

        }
}
