package org.firstinspires.ftc.teamcode.subsystem.intake;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    DcMotorEx topRoller;
    DcMotorEx bottomRoller;

    public Intake(HardwareMap map){
        topRoller = map.get(DcMotorEx.class, "topRoller");
        bottomRoller = map.get(DcMotorEx.class, "bottomRoller");

        bottomRoller.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setPower(double power){
        topRoller.setPower(power);
        bottomRoller.setPower(power);
    }
}
