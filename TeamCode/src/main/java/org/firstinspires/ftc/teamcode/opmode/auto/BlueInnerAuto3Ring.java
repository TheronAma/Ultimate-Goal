package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.opmode.OpModeBase;
import org.firstinspires.ftc.teamcode.opmode.OpModeConstants;
import org.firstinspires.ftc.teamcode.subsystem.drivetrain.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystem.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystem.shooter.Barrel;
import org.firstinspires.ftc.teamcode.subsystem.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.wobblegoal.Wobble;

import java.util.Vector;

@Config
@Autonomous()
public class BlueInnerAuto3Ring extends OpModeBase {

    public static double TARGET_RPM = 4200;
    public static double TARGET_ANGLE = 20;
    public static double ANGLE_OFFSET = 2;

    public static int TIME_BETWEEN_SHOTS = 666;

    private enum Rings {
        NO_RING,
        ONE_RING,
        FOUR_RING
    }

    private DriveTrain drive;
    private Shooter shooter;
    private Barrel barrel;
    private Intake intake;
    private Wobble wobble;

    private Rings rings;

    public static Pose2d START_POSE = new Pose2d(-63.0, 18.0, 0.0);
    public static Pose2d SHOOT_POSE = new Pose2d(
            -20.0,15.0,Math.atan2(OpModeConstants.BLUE_HIGH_GOAL.getY()-15,
            OpModeConstants.BLUE_HIGH_GOAL.getX()-(-20.0)) + Math.toRadians(ANGLE_OFFSET));
    public static Pose2d NO_RING_WOBBLE_POSE = new Pose2d();
    public static Pose2d ONE_RING_WOBBLE_POSE = new Pose2d();
    public static Pose2d FOUR_RING_WOBBLE_POSE = new Pose2d(64,37,Math.PI/2);

    public static Pose2d FOUR_RING_PICK_UP_POSE = new Pose2d(-24.0,22,Math.PI/2);

    public void runOpMode() throws InterruptedException {

        drive = new DriveTrain(hardwareMap);
        shooter = new Shooter(hardwareMap);
        barrel = new Barrel(hardwareMap);
        intake = new Intake(hardwareMap);
        wobble = new Wobble(hardwareMap);

        sleepTimer = new ElapsedTime();

        //set original drive estimate
        drive.setPoseEstimate(START_POSE);

        //intialize servos
        shooter.setAngle(TARGET_ANGLE);
        barrel.retract();
        barrel.raise();
        wobble.close();

        //calculate trajectories
        Trajectory shootTraj = drive.trajectoryBuilder(START_POSE, 0.0)
                .addTemporalMarker(0.1,()->{
                    shooter.setRpm(TARGET_RPM);
                })
                .splineToLinearHeading(SHOOT_POSE, 0.0)
                .build();

        Trajectory wobbleTraj4 = drive.trajectoryBuilder(SHOOT_POSE, SHOOT_POSE.getHeading() + Math.toRadians(ANGLE_OFFSET))
                .splineToLinearHeading(FOUR_RING_WOBBLE_POSE, Math.PI/2)
                .addTemporalMarker(0.1,()->shooter.setPower(0))
                .addTemporalMarker(2.5,()->wobble.extend())
                .build();

        rings = Rings.FOUR_RING;



        waitForStart();
        if(isStopRequested()){
            return;
        }
        wobble.retract();

        drive.followTrajectoryAsync(shootTraj);
        while(drive.isBusy() && opModeIsActive()){ drive.update(); }
        waitTimeMillis(600);

        for(int i = 0; i < 3; i++){
            barrel.extend();
            waitTimeMillis(TIME_BETWEEN_SHOTS/2);
            barrel.retract();
            waitTimeMillis(TIME_BETWEEN_SHOTS/2);
        }

        barrel.lower();
        waitTimeMillis(300);
/*
        switch(rings){
            case FOUR_RING:
                drive.followTrajectoryAsync(drive.trajectoryBuilder(drive.getPoseEstimate(),true)
                        .splineToLinearHeading(FOUR_RING_PICK_UP_POSE,Math.PI/2)
                        .addTemporalMarker(0.5,()->intake.setPower(1))
                        .build()
                );
                while(opModeIsActive() && drive.isBusy()){
                    drive.update();
                }
                waitTimeMillis(700);
                drive.followTrajectoryAsync(drive.trajectoryBuilder(drive.getPoseEstimate())
                        .forward(5)
                        .build()
                );
                while(opModeIsActive() && drive.isBusy()){
                    drive.update();
                }
                waitTimeMillis(700);
                drive.followTrajectoryAsync(drive.trajectoryBuilder(drive.getPoseEstimate())
                        .forward(5)
                        .build()
                );
                while(opModeIsActive()&&drive.isBusy()){
                    drive.update();
                }
                break;
        }

 */

        drive.followTrajectoryAsync(wobbleTraj4);
        while(drive.isBusy() && opModeIsActive()) { drive.update(); }
        wobble.open();
        waitTimeMillis(400);

        Trajectory goHome = drive.trajectoryBuilder(drive.getPoseEstimate(),true)
                .splineTo(new Vector2d(START_POSE.getX()+24,START_POSE.getY()),START_POSE.getHeading() + Math.PI)
                .addTemporalMarker(0.2,()->wobble.retract())
                .build();

        drive.followTrajectoryAsync(goHome);
        while(opModeIsActive() && drive.isBusy()){
            drive.update();
        }

        while(opModeIsActive()){

        }


    }



}
