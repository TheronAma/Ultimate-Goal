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
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Shooter {
        private DcMotorEx flywheel;
        private Servo flap;

        private double lastPos;
        private double currentPos;
        private double lastTime;
        private ElapsedTime timer;

        private double targetVelocity;

        private FtcDashboard dashboard;

        public Shooter(HardwareMap hwMap){
                flywheel = hwMap.get(DcMotorEx.class,"flywheel");
                flap = hwMap.get(Servo.class,"flap");

                lastPos = flywheel.getCurrentPosition();
                timer = new ElapsedTime();
/*
                flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                        new PIDFCoefficients(
                                Const.MOTOR_VELO_PID.p,
                                Const.MOTOR_VELO_PID.i,
                                Const.MOTOR_VELO_PID.d,
                                Const.MOTOR_VELO_PID.f));

 */

                dashboard = FtcDashboard.getInstance();
                dashboard.setTelemetryTransmissionInterval(10);
                lastTime = timer.milliseconds();

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
                currentPos = flywheel.getCurrentPosition();
                packet.put("Motor Velocity from method",flywheel.getVelocity());
                dashboard.sendTelemetryPacket(packet);

                lastPos = currentPos;
                lastTime = timer.seconds();
        }


}
