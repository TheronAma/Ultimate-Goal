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

@TeleOp()
public class TestTeleOp extends LinearOpMode {

    private enum State {
        MAIN_DRIVER_CONTROL,
        SHOOTING,
        LINE_UP
    }

    private State state;

    private DriveTrain drive;
    private Intake intake;
    private Barrel barrel;
    private Shooter shooter;



    private boolean a2WasPressed = false;
    private boolean b2WasPressed = false;
    private boolean rbumber2Waspressed = false;

    private boolean shooterRunning = false;


    public void runOpMode(){
        drive = new DriveTrain(hardwareMap);
        intake = new Intake(hardwareMap);
        barrel = new Barrel(hardwareMap);
        shooter = new Shooter(hardwareMap);

        state = State.MAIN_DRIVER_CONTROL;

        waitForStart();

        while(opModeIsActive()){

            //main dt code
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            if(gamepad2.a && !a2WasPressed){
                a2WasPressed = true;
                barrel.toggleArm();
            } else {
                a2WasPressed = false;
            }

            if(gamepad2.b && !b2WasPressed){
                b2WasPressed = true;
                barrel.togglePusher();
            } else {
                b2WasPressed = false;
            }

            intake.setPower(gamepad2.right_trigger - gamepad2.left_trigger);

            if(!rbumber2Waspressed && gamepad2.right_bumper){
                rbumber2Waspressed = true;
                shooterRunning = !shooterRunning;
            } else {
                rbumber2Waspressed = false;
            }

            if(shooterRunning){
                shooter.setRpm(Const.SHOOTER_REGULAR_RPM);
            } else {
                shooter.setRpm(0);
            }

            drive.update();

        }

    }

}
