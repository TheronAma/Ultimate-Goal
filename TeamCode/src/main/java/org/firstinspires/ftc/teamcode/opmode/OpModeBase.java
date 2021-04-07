package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.drivetrain.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystem.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystem.shooter.Shooter;

//TODO Look into command based opmodes
public abstract class OpModeBase extends LinearOpMode {

    public ElapsedTime sleepTimer;
    public InterpLUT controlPoints;

    public void waitTimeMillis(double t){
        sleepTimer.reset();
        while(sleepTimer.milliseconds() < t){
            update();
        }
    }

    public void update() {

    }



}
