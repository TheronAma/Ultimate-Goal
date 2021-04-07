package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.opmode.OpModeBase;
import org.firstinspires.ftc.teamcode.opmode.OpModeConstants;
import org.firstinspires.ftc.teamcode.subsystem.drivetrain.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystem.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystem.shooter.Barrel;
import org.firstinspires.ftc.teamcode.subsystem.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.wobblegoal.Wobble;

@Config
@TeleOp()
public class BlueTeleOp extends LinearOpMode {

    private enum State {
        DRIVER_CONTROL,
        AUTOMATIC
    }

    private State state;

    private Wobble wobble;
    private DriveTrain drive;
    private Intake intake;
    private Barrel barrel;
    private Shooter shooter;
    private Servo camServo;

    private boolean a2WasPressed = false;
    private boolean b2WasPressed = false;
    private boolean rbumber2Waspressed = false;
    private boolean x2WasPressed = false;
    private boolean y2WasPressed = false;

    private boolean shooterRunning = false;
    private double shooterRpm = 4200;
    private double shooterAngle = 22;

    private int shotNum = 0;
    private double target = 0;
    private double heading = 0;

    private Vector2d POWER_POSE = new Vector2d(-8,-34);

    public static double firstShotAngle = Math.toRadians(8);
    public static double secondShotAngle = Math.toRadians(1);
    public static double thirdShotAngle = Math.toRadians(-5.5);

    private Vector2d POWER_SHOT_1_POS = new Vector2d(-3,-19.5);
    private Vector2d POWER_SHOT_2_POS = new Vector2d(-6,-31.5);
    private Vector2d POWER_SHOT_3_POS = new Vector2d(-6,-40.5);

    public void runOpMode(){
        drive = new DriveTrain(hardwareMap);
        intake = new Intake(hardwareMap);
        barrel = new Barrel(hardwareMap);
        shooter = new Shooter(hardwareMap);
        wobble = new Wobble(hardwareMap);

        drive.setPoseEstimate(OpModeConstants.AutoEndPose);

        wobble.retract();

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

            if(gamepad1.b && !x2WasPressed){
                x2WasPressed = true;
                wobble.toggleExtend();
            } else if(!gamepad1.b){
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

            switch(state){
                case DRIVER_CONTROL:
                    drive.setDrivePower(
                            new Pose2d(
                                    -gamepad1.left_stick_y * multiplier,
                                    -gamepad1.left_stick_x * multiplier,
                                    -gamepad1.right_stick_x * multiplier
                            )
                    );
                    if(gamepad1.dpad_down){
                        switch(shotNum){
                            case 0:
                                drive.setPoseEstimate(new Pose2d(0,0,0.0));
                                drive.followTrajectoryAsync(drive.trajectoryBuilder(drive.getPoseEstimate())
                                        .lineToLinearHeading(new Pose2d(POWER_POSE.getX(),POWER_POSE.getY(),0.0))
                                        .build()
                                );
                                state = State.AUTOMATIC;
                                shotNum++;
                                break;
                            case 1:
                                heading = drive.getPoseEstimate().getHeading();
                                target = firstShotAngle-heading;
                                if(target > Math.PI){
                                    target -= 2* Math.PI;
                                }

                                if(target < -
                                        Math.PI){
                                    target += 2 * Math.PI;
                                }
                                drive.turnAsync(firstShotAngle);
                                state = State.AUTOMATIC;
                                shotNum++;
                                break;
                            case 2:
                                /*
                                drive.followTrajectoryAsync(drive.trajectoryBuilder(drive.getPoseEstimate())
                                        .lineToLinearHeading(new Pose2d(POWER_SHOT_2_POS.getX(),POWER_SHOT_2_POS.getY(),0.0))
                                        .build()
                                );
                                 */
                                heading = drive.getPoseEstimate().getHeading();
                                target = secondShotAngle-heading;
                                if(target > Math.PI){
                                    target -= 2* Math.PI;
                                }

                                if(target < -
                                        Math.PI){
                                    target += 2 * Math.PI;
                                }

                                drive.turnAsync(target);
                                state = State.AUTOMATIC;
                                shotNum++;
                                break;
                            case 3:
                                /*
                                drive.followTrajectoryAsync(drive.trajectoryBuilder(drive.getPoseEstimate())
                                        .lineToLinearHeading(new Pose2d(POWER_SHOT_3_POS.getX(),POWER_SHOT_3_POS.getY(),0.0))
                                        .build()
                                );

                                 */
                                heading = drive.getPoseEstimate().getHeading();
                                target = thirdShotAngle - heading;
                                if(target > Math.PI){
                                    target -= 2* Math.PI;
                                }

                                if(target < -Math.PI){
                                    target += 2 * Math.PI;
                                }

                                drive.turnAsync(target);
                                state = State.AUTOMATIC;
                                shotNum = 3;
                                break;
                            default:
                                shotNum++;
                        }
                    }
                case AUTOMATIC:
                    // If drive finishes its task, cede control to the driver
                    if (!drive.isBusy()) {
                        state = State.DRIVER_CONTROL;
                    }
                    break;

            }
            drive.update();
            shooter.updatePIDCoeffs();
            telemetry.addData("Position",drive.getPoseEstimate());
            telemetry.addData("Flywheel Velo",shooterRpm);
            telemetry.update();
        }
    }


}