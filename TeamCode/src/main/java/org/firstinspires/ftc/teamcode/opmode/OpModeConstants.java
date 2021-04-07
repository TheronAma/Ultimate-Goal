package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.util.InterpLUT;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Vector;

@Config
public class OpModeConstants {
    public static Vector2d RED_HIGH_GOAL = new Vector2d(72,-36);
    public static Vector2d BLUE_HIGH_GOAL = new Vector2d(72,36);

    public static Pose2d AutoEndPose = new Pose2d();
    public static double INTAKE_POWER = 0.9;


}
