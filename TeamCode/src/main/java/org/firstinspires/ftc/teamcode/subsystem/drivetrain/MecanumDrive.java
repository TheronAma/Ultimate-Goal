package org.firstinspires.ftc.teamcode.subsystem.drivetrain;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;


public class MecanumDrive {

    /*
        These are the drive constants and constraints.
        These will be used for calculations required in order
        to follow trajectories, and for feedforward control.
     */

    public static final double TICKS_PER_REV = 537.6;
    public static final double MAX_RPM = 312;

    public static final boolean RUN_USING_ENCODER = true;

    public static final double WHEEL_RADIUS = 2;
    public static final double GEAR_RATIO = 1;
    public static final double MAX_VELOCITY = 312 * GEAR_RATIO * WHEEL_RADIUS * 2 * Math.PI / 60.0;
    public static double TRACK_WIDTH = 15;

    public static double kV = 1.0 / (MAX_VELOCITY); //ratio between motor power and velocity
    public static double kAtrans = 0; //ratio between motor power and translational acceleration
    public static double kArot = 0; //ratio between motor power and rotational acceleration
    public static double kD = 0; //constant to characterize deceleration curve of motor
    public static double kStatic = 0;//amount of power required to overcome static friction

    public static DriveConstraints BASE_CONSTRAINTS = new DriveConstraints(
            30.0, 30.0, 0.0,
            Math.toRadians(180.0), Math.toRadians(180.0), 0.0
    );

    public static PIDCoefficients translationalPID = new PIDCoefficients(0,0,0);
    public static PIDCoefficients headingPID = new PIDCoefficients(0,0,0);



    public MecanumDrive(HardwareMap hwMap){

        

    }


}
