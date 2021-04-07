package org.firstinspires.ftc.teamcode.subsystem.wobblegoal;

import com.acmerobotics.dashboard.config.Config;

@Config
public class WobbleConstants {
    public static double CLAW_OPEN_POSITION = 0;
    public static double CLAW_CLOSE_POSITION = 0.7;

    public static double TICKS_PER_REV = 1425.2;
    public static int TARGET_ARM_POS = (int)-TICKS_PER_REV / 10 * 3;
}
