package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.opmode.OpModeBase;
import org.firstinspires.ftc.teamcode.opmode.OpModeConstants;
import org.firstinspires.ftc.teamcode.subsystem.detector.HighGoalDetector;
import org.firstinspires.ftc.teamcode.subsystem.drivetrain.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystem.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystem.shooter.Barrel;
import org.firstinspires.ftc.teamcode.subsystem.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.wobblegoal.Wobble;

import java.lang.annotation.ElementType;

@TeleOp()
public class BlueTeleOpTest extends LinearOpMode {

    private enum State {
        DRIVER_CONTROL,
        POWER_SHOT,
        HIGH_GOAL,
        AUTOMATIC
    }

    private State state;


    private Wobble wobble;
    private DriveTrain drive;
    private Intake intake;
    private Barrel barrel;
    private Shooter shooter;

    private boolean lastLeftBump = false;
    private boolean a2WasPressed = false;
    private boolean b2WasPressed = false;
    private boolean rbumber2Waspressed = false;
    private boolean x2WasPressed = false;
    private boolean y2WasPressed = false;

    private boolean shooterRunning = false;
    private double shooterRpm = 4150;
    private double shooterAngle = 22;

    private HighGoalDetector detector;

    private int shotNum = 0;
    private int highGoalState = 0;

    private Vector2d POWER_SHOT_1_POS = new Vector2d(-6,-52);
    private Vector2d POWER_SHOT_2_POS = new Vector2d(-6,-61);
    private Vector2d POWER_SHOT_3_POS = new Vector2d(-6,-72);

    private PIDCoefficients HEADING_PID = new PIDCoefficients(3,0,0.1);
    private PIDFController turnController = new PIDFController(HEADING_PID,0);

    public void runOpMode(){
        ElapsedTime t = new ElapsedTime();

        drive = new DriveTrain(hardwareMap);
        intake = new Intake(hardwareMap);
        barrel = new Barrel(hardwareMap);
        shooter = new Shooter(hardwareMap);
        wobble = new Wobble(hardwareMap);
        detector = new HighGoalDetector(hardwareMap,"camera");

        drive.setPoseEstimate(OpModeConstants.AutoEndPose);

        shooter.setAngle(23);
        state = State.DRIVER_CONTROL;

        waitForStart();

        while(opModeIsActive()){
            //main dt code

            double multiplier = 1;
            if(gamepad1.right_bumper){
                multiplier = 0.5;
            }

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

            if(gamepad1.x && !x2WasPressed){
                x2WasPressed = true;
                wobble.toggleExtend();
            } else if(!gamepad1.x){
                x2WasPressed = false;
            }

            if(gamepad1.y && !y2WasPressed){
                y2WasPressed = true;
                wobble.toggleOpen();
            } else if(!gamepad1.y){
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
                shooterRpm = 3600;
                shooterAngle = 0;
                shooter.setAngle(shooterAngle);
            }

            if(gamepad2.dpad_up){
                shooterRpm = 4150;
                shooterAngle = 23;
                shooter.setAngle(shooterAngle);
            }

            if(shooterRunning){
                shooter.setRpm(shooterRpm);
                shooter.setAngle(shooterAngle);
            } else {
                shooter.setRpm(0);
            }

            double error = drive.getPoseEstimate().getHeading();
            if(error > Math.PI){
                error -= Math.PI * 2;
            }

            switch(state){
                case DRIVER_CONTROL:

                    if(gamepad1.left_bumper && !lastLeftBump){
                        turnController.setTargetPosition(0.0);
                        state = State.HIGH_GOAL;
                    }

                    lastLeftBump = gamepad1.left_bumper;

                    drive.setDrivePower(
                            new Pose2d(
                                    -gamepad1.left_stick_y * multiplier,
                                    -gamepad1.left_stick_x * multiplier,
                                    -gamepad1.right_stick_x * multiplier
                            )
                    );

                    if(gamepad1.a){
                        state = State.POWER_SHOT;
                        shooter.setRpm(3750);
                        shooter.setAngle(0);
                        shooterRunning = true;
                        barrel.raise();

                        drive.setPoseEstimate(new Pose2d(0,0,0.0));
                        shotNum = 0;
                    }
                    break;
                case HIGH_GOAL:
                    drive.setDrivePower(
                            new Pose2d(
                                    -gamepad1.left_stick_y * multiplier,
                                    -gamepad1.left_stick_x * multiplier,
                                    turnController.update(error) + -gamepad1.right_stick_x * multiplier
                            )
                    );
                    if(gamepad1.left_bumper && !lastLeftBump){
                        state = State.DRIVER_CONTROL;
                    }
                    break;
                case AUTOMATIC:
                    // If drive finishes its task, cede control to the driver
                    if (!drive.isBusy()) {
                        state = State.DRIVER_CONTROL;
                    }
                    break;
                case POWER_SHOT:
                    switch(shotNum){
                        case 0:
                            drive.followTrajectoryAsync(drive.trajectoryBuilder(drive.getPoseEstimate())
                                    .lineToLinearHeading(new Pose2d(POWER_SHOT_1_POS.getX(),POWER_SHOT_1_POS.getY(),0.0))
                                    .build());
                            shotNum++;
                            break;
                        case 1:
                            if(drive.isBusy() && opModeIsActive()){
                                drive.update();
                            } else {
                                barrel.extend();
                                drive.followTrajectoryAsync(drive.trajectoryBuilder(drive.getPoseEstimate())
                                        .addTemporalMarker(0.2, ()->barrel.retract())
                                        .lineToLinearHeading(new Pose2d(POWER_SHOT_2_POS.getX(),POWER_SHOT_2_POS.getY(),0.0))
                                        .build());
                                shotNum++;
                            }
                            break;
                        case 2:
                            if(drive.isBusy() && opModeIsActive()){
                                drive.update();
                            } else {
                                barrel.extend();
                                drive.followTrajectoryAsync(drive.trajectoryBuilder(drive.getPoseEstimate())
                                        .addTemporalMarker(0.2, ()->barrel.retract())
                                        .lineToLinearHeading(new Pose2d(POWER_SHOT_3_POS.getX(),POWER_SHOT_3_POS.getY(),0.0))
                                        .build());
                                shotNum++;
                            }
                            break;
                        case 3:
                            if(drive.isBusy() && opModeIsActive()) {
                                drive.update();
                            } else {
                                barrel.extend();
                                t.reset();
                                shotNum++;
                            }
                            break;
                        case 4:
                            drive.setDrivePower(
                                    new Pose2d(
                                            -gamepad1.left_stick_y * multiplier,
                                            -gamepad1.left_stick_x * multiplier,
                                            -gamepad1.right_stick_x * multiplier
                                    )
                            );
                            if(t.milliseconds() > 200 && opModeIsActive()){
                                barrel.retract();
                            }
                            shotNum = 0;
                            state = State.DRIVER_CONTROL;
                            break;
                    }
                    break;

            }
            telemetry.addData("State",state);
            telemetry.addData("Heading",drive.getPoseEstimate().getHeading());
            telemetry.update();
            drive.update();


        }
    }


}