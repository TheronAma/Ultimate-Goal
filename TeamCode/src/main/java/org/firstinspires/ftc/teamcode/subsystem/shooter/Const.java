package org.firstinspires.ftc.teamcode.subsystem.shooter;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Const {

    public static double SHOOTER_MAX_RPM = 5400;
    public static double SHOOTER_REGULAR_RPM = 4800;
    public static int TICKS_PER_REV = 28;
    public static double SHOOTER_HEADING_OFFSET = 0;

    public static double FLAP_MIN_POS = 0.4;
    public static double FLAP_MAX_POS = 0.6;
    public static double FLAP_AUTO_ANGLE = 20;
    public static double FLAP_RANGE = FLAP_MAX_POS - FLAP_MIN_POS;

    public static double BARREL_TOP_POS = 0.5;
    public static double BARREL_BOTTOM_POS = 0.5;

    public static double PUSHER_EXTEND_POS = 0.5;
    public static double PUSHER_RETRACT_POS = 0.5;

    public static int PUSHER_MOVE_TIME = 200;

    public static double rpmToTicksPerRev(double rpm){
        return rpm * 28.0 / 60.0;
    }

}
