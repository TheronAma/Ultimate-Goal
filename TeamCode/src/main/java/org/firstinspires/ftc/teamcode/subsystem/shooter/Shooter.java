package org.firstinspires.ftc.teamcode.subsystem.shooter;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Shooter {
        DcMotorEx flywheel;
        Servo flap;

        double currentTime;
        double lastTime;

        double offset;

        double velocity;
        double currentPosition;
        double lastPosition;

        double targetRpm = 0;

        public static double TICKS_PER_REV = 28;

        public static double kV = 0.01;
        PIDCoefficients velocityPID = new PIDCoefficients(1,0,0);

        PIDFController velocityController;

        /*



         */

        public Shooter(HardwareMap hwMap){
                flywheel = hwMap.get(DcMotorEx.class,"flywheel");
                flap = hwMap.get(Servo.class,"flap");
                velocityController = new PIDFController(velocityPID,kV);

                offset = flywheel.getCurrentPosition();

                currentTime = System.nanoTime() * Math.pow(10,9);

                lastPosition = 0;
                currentPosition = 0;

        }

        public void setPower(double power){
                flywheel.setPower(power);
        }

        public void setVelocity(double rpm){
                targetRpm = rpm;
                velocityController.setTargetPosition(rpm);
        }

        public void setAngle(double degrees){

        }
        /*
        The update() method updates the state variables of the flywheel, which include velocity and position
        It also updates the PID controller for the velocity of the flywheel
        It also records the time
        */
        public void update(){
                currentPosition = flywheel.getCurrentPosition() - offset;
                currentTime = System.nanoTime() * Math.pow(10,9);

                velocity = ((currentPosition-lastPosition)/28*60)/(currentTime-lastTime);

                double power = velocityController.update(velocity);
                flywheel.setPower(power);

                lastPosition = currentPosition;
                lastTime = currentTime;
        }

        public double getVelocity(){
                return velocity;
        }
}
