package org.firstinspires.ftc.teamcode.subsystem.drivetrain;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


@Config
public class DriveConstants {

    /*
     * These are motor constants that should be listed online for your motors.
     */
    public static final double TICKS_PER_REV = 383.6;
    public static final double MAX_RPM = 435;

    public static final boolean RUN_USING_ENCODER = false;
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(25, 0, 0, 13);

    public static double WHEEL_RADIUS = 48/25.4; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (motor) speed
    public static double TRACK_WIDTH = 14.3; // in



    //11.7V 63.92 in/s
    //11.5V 62.8385 in/s
    //14.11 86.9813 in/s

    //12V 65.58 in/s <-- guess

    //This stuff is at 13.4V
    public static double START_KV = 0.0135;
    public static double START_KA = 0.0015;
    public static double START_KS = 0.015;

    public static double kV = 1/rpmToVelocity(MAX_RPM);
    public static double kA = 0;
    public static double kStatic = 0;



    public static DriveConstraints BASE_CONSTRAINTS = new DriveConstraints(
            55.0, 45.0, 0.0,
            Math.toRadians(180.0), Math.toRadians(180.0), 0.0
    );


    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }

    public static double getMotorVelocityF(double ticksPerSecond) {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 / ticksPerSecond;
    }
}
