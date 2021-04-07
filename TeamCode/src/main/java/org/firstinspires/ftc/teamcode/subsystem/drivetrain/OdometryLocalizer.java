package org.firstinspires.ftc.teamcode.subsystem.drivetrain;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.arcrobotics.ftclib.hardware.SensorDistanceEx;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;
import java.util.List;

/*
 * Localizer from Roadrunner quickstart
 */
public class OdometryLocalizer extends ThreeTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 1.378 / 2 * 96 / 92; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 8.865 * 96 / 92; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = -7; // in; offset of the lateral wheel
    public static double LATERAL_OFFSET = -2;


    private DcMotorEx leftEncoder, rightEncoder, frontEncoder;

    public OdometryLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, LATERAL_OFFSET, Math.toRadians(90)) // front
        ));

        leftEncoder = hardwareMap.get(DcMotorEx.class, "fr");
        rightEncoder = hardwareMap.get(DcMotorEx.class, "bl");
        frontEncoder = hardwareMap.get(DcMotorEx.class, "fl");
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }


    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                -encoderTicksToInches(leftEncoder.getCurrentPosition()),
                encoderTicksToInches(rightEncoder.getCurrentPosition()),
                -encoderTicksToInches(frontEncoder.getCurrentPosition() * 96 / 98)
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {

        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getVelocity()),
                encoderTicksToInches(rightEncoder.getVelocity()),
                encoderTicksToInches(frontEncoder.getVelocity())
        );
    }
}