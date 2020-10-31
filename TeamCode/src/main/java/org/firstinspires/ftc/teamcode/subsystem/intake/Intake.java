package org.firstinspires.ftc.teamcode.subsystem.intake;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    DcMotorEx frontIntake;
    DcMotorEx backIntake;

    public Intake(HardwareMap map){
        frontIntake = map.get(DcMotorEx.class, "frontIntake");
        backIntake = map.get(DcMotorEx.class, "backIntake");
    }

    public void setPower(double power){
        frontIntake.setPower(power);
        backIntake.setPower(power);
    }
}
