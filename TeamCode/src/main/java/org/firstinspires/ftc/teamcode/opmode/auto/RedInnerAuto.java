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
import org.firstinspires.ftc.teamcode.subsystem.detector.RingDetector;
import org.firstinspires.ftc.teamcode.subsystem.detector.RingPipeline;
import org.firstinspires.ftc.teamcode.subsystem.drivetrain.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystem.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystem.shooter.Barrel;
import org.firstinspires.ftc.teamcode.subsystem.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.wobblegoal.Wobble;

import java.util.Vector;

@Config
@Autonomous()
public class RedInnerAuto extends OpModeBase {

    public static double TARGET_RPM = 3950;
    public static double TARGET_ANGLE = 20;
    public static double ANGLE_OFFSET = 3;
    public static double DELAY = 1;

    public static int TIME_BETWEEN_SHOTS = 500;

    private DriveTrain drive;
    private Shooter shooter;
    private Barrel barrel;
    private Intake intake;
    private Wobble wobble;
    private RingDetector detector;

    private RingPipeline.Rings rings = RingPipeline.Rings.FOUR;

    public static Pose2d START_POSE = new Pose2d(-63.0, -18.0, 0.0);
    public static Pose2d DETECT_POSE = new Pose2d(-42,-12, -Math.PI/4);
    public static Pose2d SHOOT_POSE = new Pose2d(
            -20.0,-8,Math.atan2(OpModeConstants.RED_HIGH_GOAL.getY()+15,
            OpModeConstants.RED_HIGH_GOAL.getX()-(-8)) + Math.toRadians(ANGLE_OFFSET));
    public static Pose2d NO_RING_WOBBLE_POSE = new Pose2d(13,-36,-Math.PI/2);
    public static Pose2d ONE_RING_WOBBLE_POSE = new Pose2d(33,-15,-Math.PI/2);
    public static Pose2d FOUR_RING_WOBBLE_POSE = new Pose2d(58,-36,-Math.PI/2);

    //public static Pose2d FOUR_RING_PICK_UP_POSE = new Pose2d(-24.0,-22,Math.PI/2);

    public void runOpMode() throws InterruptedException {

        drive = new DriveTrain(hardwareMap);
        shooter = new Shooter(hardwareMap);
        barrel = new Barrel(hardwareMap);
        intake = new Intake(hardwareMap);
        wobble = new Wobble(hardwareMap);
        detector = new RingDetector(hardwareMap,"camera");

        detector.init();

        sleepTimer = new ElapsedTime();

        //set original drive estimate
        drive.setPoseEstimate(START_POSE);

        //intialize servos
        shooter.setAngle(TARGET_ANGLE);
        barrel.retract();
        barrel.raise();
        wobble.close();

        //calculate trajectories
        Trajectory detectTraj = drive.trajectoryBuilder(START_POSE, 0.0)
                .addTemporalMarker(0.4,()->{
                    shooter.setRpm(TARGET_RPM);
                })
                .splineToLinearHeading(DETECT_POSE, -Math.PI/4)
                .build();

        Trajectory shootTraj = drive.trajectoryBuilder(DETECT_POSE, DETECT_POSE.getHeading())
                .splineToLinearHeading(SHOOT_POSE, 0.0)
                .build();

        Trajectory wobbleTraj4 = drive.trajectoryBuilder(SHOOT_POSE, SHOOT_POSE.getHeading() + Math.toRadians(ANGLE_OFFSET))
                .splineToLinearHeading(FOUR_RING_WOBBLE_POSE, Math.PI/2)
                .addTemporalMarker(0.1,()->shooter.setPower(0))
                .addTemporalMarker(2.5,()->wobble.extend())
                .build();

        Trajectory wobbleTraj1 = drive.trajectoryBuilder(SHOOT_POSE,SHOOT_POSE.getHeading() + Math.toRadians(ANGLE_OFFSET))
                .splineToLinearHeading(ONE_RING_WOBBLE_POSE, Math.PI/2)
                .addTemporalMarker(0.1,()->shooter.setPower(0))
                .addTemporalMarker(1.5,()->wobble.extend())
                .build();

        Trajectory wobbleTraj0 = drive.trajectoryBuilder(SHOOT_POSE,SHOOT_POSE.getHeading() + Math.toRadians(ANGLE_OFFSET))
                .splineToLinearHeading(NO_RING_WOBBLE_POSE, Math.PI/2)
                .addTemporalMarker(0.1,()->shooter.setPower(0))
                .addTemporalMarker(1.5,()->wobble.extend())
                .build();



        while(!isStarted() && !isStopRequested()){
            rings = detector.getRings();
            telemetry.addData("Rings: ",rings);
            telemetry.update();
        }

        waitForStart();
        if(isStopRequested()){
            return;
        }

        wobble.retract();

        drive.followTrajectoryAsync(detectTraj);
        while(drive.isBusy() && opModeIsActive()){ drive.update(); shooter.updatePIDCoeffs(); }
        sleepTimer.reset();
        rings = detector.getRings();
        telemetry.addData("Rings: ",rings);
        telemetry.update();
        detector.close();



        drive.followTrajectoryAsync(shootTraj);
        while(drive.isBusy() && opModeIsActive()){ drive.update(); shooter.updatePIDCoeffs(); }

        for(int i = 0; i < 3; i++){
            barrel.extend();
            waitTimeMillis(TIME_BETWEEN_SHOTS/2);
            barrel.retract();
            waitTimeMillis(TIME_BETWEEN_SHOTS/2);
        }
        barrel.lower();

        switch(rings){
            case FOUR:
                drive.followTrajectoryAsync(
                        drive.trajectoryBuilder(drive.getPoseEstimate())
                                .splineToLinearHeading(new Pose2d(-26,-22,-Math.PI/2),-Math.PI/2)
                                .build()
                ); while(!isStopRequested() && drive.isBusy()){ drive.update(); shooter.updatePIDCoeffs(); }

                intake.setPower(OpModeConstants.INTAKE_POWER);
                drive.followTrajectoryAsync(
                        drive.trajectoryBuilder(drive.getPoseEstimate())
                                .forward(8)
                                .build()
                ); while(!isStopRequested() && drive.isBusy()){ drive.update(); shooter.updatePIDCoeffs(); }
                intake.setPower(-1);
                waitTimeMillis(200);

                intake.setPower(OpModeConstants.INTAKE_POWER);
                drive.followTrajectoryAsync(
                        drive.trajectoryBuilder(drive.getPoseEstimate())
                                .forward(4)
                                .build()
                ); while(!isStopRequested() && drive.isBusy()){ drive.update(); shooter.updatePIDCoeffs(); }
                intake.setPower(-1);
                waitTimeMillis(200);

                intake.setPower(OpModeConstants.INTAKE_POWER);
                drive.followTrajectoryAsync(
                        drive.trajectoryBuilder(drive.getPoseEstimate())
                                .forward(5)
                                .build()
                ); while(!isStopRequested() && drive.isBusy()){ drive.update(); shooter.updatePIDCoeffs(); }
                waitTimeMillis(300);
                intake.setPower(-1);
                waitTimeMillis(200);


                drive.followTrajectoryAsync(
                        drive.trajectoryBuilder(drive.getPoseEstimate(),true)
                                .addTemporalMarker(0.6, ()->{
                                    intake.setPower(-1);
                                    shooter.setRpm(3950);
                                    barrel.raise();
                                    barrel.retract();
                                })
                                .splineToLinearHeading(SHOOT_POSE,0.0)
                                .build()
                );
                while(!isStopRequested() && drive.isBusy()){ drive.update(); shooter.updatePIDCoeffs(); }

                intake.setPower(0);
                for(int i = 0; i< 3; i++) {
                    barrel.extend();
                    waitTimeMillis(TIME_BETWEEN_SHOTS / 2);
                    barrel.retract();
                    waitTimeMillis(TIME_BETWEEN_SHOTS / 2);
                }

                drive.followTrajectoryAsync(wobbleTraj4);
                break;
            case ONE:
                drive.followTrajectoryAsync(
                        drive.trajectoryBuilder(drive.getPoseEstimate())
                                .splineToLinearHeading(new Pose2d(-26,-22,-Math.PI/2),-Math.PI/2)
                                .build()
                ); while(!isStopRequested() && drive.isBusy()){ drive.update(); shooter.updatePIDCoeffs();}

                intake.setPower(OpModeConstants.INTAKE_POWER);
                drive.followTrajectoryAsync(
                        drive.trajectoryBuilder(drive.getPoseEstimate())
                                .forward(8)
                                .build()
                );
                while(!isStopRequested() && drive.isBusy()){ drive.update(); shooter.updatePIDCoeffs();}
                telemetry.addLine("WTF IS GOING ON");
                telemetry.update();

                drive.followTrajectoryAsync(
                        drive.trajectoryBuilder(drive.getPoseEstimate(),true)
                                .splineToLinearHeading(new Pose2d(
                                        SHOOT_POSE.getX(),
                                        SHOOT_POSE.getY(),
                                        SHOOT_POSE.getHeading() - Math.toRadians(1.5)
                                ),0.0)
                                .addTemporalMarker(0.1, ()->{
                                    shooter.setRpm(3900);
                                    intake.setPower(1);
                                })
                                .addTemporalMarker(0.35, ()->{
                                    intake.setPower(-0.7);
                                })
                                .addTemporalMarker(0.6, ()->{
                                    intake.setPower(-1);
                                    barrel.raise();
                                    barrel.retract();
                                })
                                .build()
                );

                telemetry.addLine("MADE TRAJECTORy");
                telemetry.update();

                while(!isStopRequested() && drive.isBusy()){ drive.update(); shooter.updatePIDCoeffs(); }
                telemetry.addLine("SHOOTING");
                telemetry.update();

                intake.setPower(0);
                barrel.extend();
                waitTimeMillis(TIME_BETWEEN_SHOTS/2);
                barrel.retract();
                waitTimeMillis(TIME_BETWEEN_SHOTS/2);

                drive.followTrajectoryAsync(wobbleTraj1);
                break;
            case ZERO:
                drive.followTrajectoryAsync(wobbleTraj0);
                break;
        }

        while(drive.isBusy() && opModeIsActive()) { drive.update(); shooter.updatePIDCoeffs();}
        wobble.open();
        waitTimeMillis(300);

        Trajectory goHome = drive.trajectoryBuilder(drive.getPoseEstimate(),true)
                .splineTo(new Vector2d(16,-10),Math.PI)
                .addTemporalMarker(0.2,()->wobble.retract())
                .build();

        drive.followTrajectoryAsync(goHome);
        while(opModeIsActive() && drive.isBusy()){
            drive.update() ;
            shooter.updatePIDCoeffs();
        }
        waitTimeMillis(100);
        OpModeConstants.AutoEndPose = drive.getPoseEstimate();
    }

    private void waitAsync(){
        while(!isStopRequested() && drive.isBusy()){
            drive.update();
            shooter.updatePIDCoeffs();
        }
    }



}
