package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous
public class ServoTest extends LinearOpMode {
    public static double target = 0.5;
    private Servo servo;
    public void runOpMode(){
        servo = hardwareMap.get(Servo.class,"camServo");
        waitForStart();
        while(!isStopRequested()){
            servo.setPosition(target);
        }
    }
}
