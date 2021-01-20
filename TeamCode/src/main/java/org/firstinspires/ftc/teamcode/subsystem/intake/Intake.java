package org.firstinspires.ftc.teamcode.subsystem.intake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Intake {
    public static double DEFAULT_INTAKE_POWER = 0.8;

    DcMotorEx topRoller;
    DcMotorEx bottomRoller;

    public Intake(HardwareMap map){
        topRoller = map.get(DcMotorEx.class, "topRoller");
        bottomRoller = map.get(DcMotorEx.class, "bottomRoller");

        topRoller.setDirection(DcMotorSimple.Direction.REVERSE);
        bottomRoller.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setPower(double power){
        topRoller.setPower(power);
        bottomRoller.setPower(power);
    }
}
