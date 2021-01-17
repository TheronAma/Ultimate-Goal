package org.firstinspires.ftc.teamcode.subsystem.shooter;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Shooter {
        private DcMotorEx flywheel;
        private Servo flap;

        private double targetVelocity;

        private FtcDashboard dashboard;

        public Shooter(HardwareMap hwMap){
                flywheel = hwMap.get(DcMotorEx.class,"flywheel");
                flap = hwMap.get(Servo.class,"flap");

                flywheel.setDirection(DcMotorSimple.Direction.REVERSE);

                dashboard = FtcDashboard.getInstance();
                dashboard.setTelemetryTransmissionInterval(10);
        }

        public void setPower(double power){
                flywheel.setPower(power);
        }

        public void setRpm(double rpm){
                setVelocity(Const.rpmToTicksPerRev(rpm));
        }

        /**
         * Allows you to set an angel between the 0 degree and max angle positions
         * @param angle number 0 - 100 that represents the range of the flap
         */

        public void setAngle(double angle){
                if(angle >= 100){
                        flap.setPosition(Const.FLAP_MAX_POS);
                } else {
                        flap.setPosition(Const.FLAP_MIN_POS + angle * Const.FLAP_RANGE/100.);
                }

        }

        public void setVelocity(double ticksPerSecond){
                flywheel.setVelocity(ticksPerSecond);
                targetVelocity = ticksPerSecond;
        }

        public void setPosition(double position){
                flap.setPosition(position);
        }

        public double getVelocity(){
                return flywheel.getVelocity();
        }

        public double getRpm(){
                return getVelocity() / 28. * 60.;
        }

        public void sendDashTelemetry() {
                TelemetryPacket packet = new TelemetryPacket();
                packet.put("velocity", getVelocity());
                packet.put("target", targetVelocity);
                packet.put("rpm",getRpm());
                packet.put("target_rpm",targetVelocity * 60. / 28.);
                dashboard.sendTelemetryPacket(packet);
        }


}
