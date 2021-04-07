package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.subsystem.drivetrain.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystem.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystem.shooter.Barrel;
import org.firstinspires.ftc.teamcode.subsystem.shooter.Const;
import org.firstinspires.ftc.teamcode.subsystem.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.wobblegoal.Wobble;

@TeleOp()
public class TestTeleOp extends LinearOpMode {

    private enum State {
        MAIN_DRIVER_CONTROL,
        SHOOTING,
        LINE_UP,
    }

    private State state;


    private Wobble wobble;
    private DriveTrain drive;
    private Intake intake;
    private Barrel barrel;
    private Shooter shooter;

    private boolean a2WasPressed = false;
    private boolean b2WasPressed = false;
    private boolean rbumber2Waspressed = false;
    private boolean x2WasPressed = false;
    private boolean y2WasPressed = false;

    private boolean shooterRunning = false;
    private double shooterRpm = 4200;
    private double shooterAngle = 22;


    public void runOpMode(){
        drive = new DriveTrain(hardwareMap);
        intake = new Intake(hardwareMap);
        barrel = new Barrel(hardwareMap);
        shooter = new Shooter(hardwareMap);
        wobble = new Wobble(hardwareMap);

        shooter.setAngle(23);
        state = State.MAIN_DRIVER_CONTROL;

        waitForStart();

        while(opModeIsActive()){
            //main dt code

            double multiplier = 1;
            if(gamepad1.right_bumper){
                multiplier = 0.5;
            }

            drive.setDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y * multiplier,
                            -gamepad1.left_stick_x * multiplier,
                            -gamepad1.right_stick_x * multiplier
                    )
            );

            if(gamepad2.a && !a2WasPressed){
                a2WasPressed = true;
                barrel.toggleArm();
            } else if(!gamepad2.a){
                a2WasPressed = false;
            }

            if(gamepad2.b && !b2WasPressed){
                b2WasPressed = true;
                barrel.togglePusher();
            } else if(!gamepad2.b){
                b2WasPressed = false;
            }

            if(gamepad2.x && !x2WasPressed){
                x2WasPressed = true;
                wobble.toggleExtend();
            } else if(!gamepad2.x){
                x2WasPressed = false;
            }

            if(gamepad2.y && !y2WasPressed){
                y2WasPressed = true;
                wobble.toggleOpen();
            } else if(!gamepad2.y){
                y2WasPressed = false;
            }

            intake.setPower(gamepad2.right_trigger - gamepad2.left_trigger);

            if(!rbumber2Waspressed && gamepad2.right_bumper){
                rbumber2Waspressed = true;
                shooterRunning = !shooterRunning;
            } else if(!gamepad2.right_bumper){
                rbumber2Waspressed = false;
            }

            if(gamepad2.dpad_down){
                shooterRpm = 3700;
                shooterAngle = 0;
                shooter.setAngle(shooterAngle);
            }

            if(gamepad2.dpad_up){
                shooterRpm = 4100;
                shooterAngle = 23;
                shooter.setAngle(shooterAngle);
            }

            if(shooterRunning){
                shooter.setRpm(shooterRpm);
                shooter.setAngle(shooterAngle);
            } else {
                shooter.setRpm(0);
            }

            drive.update();
        }
    }
}
