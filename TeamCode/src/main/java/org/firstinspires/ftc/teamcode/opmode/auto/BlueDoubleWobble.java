package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.opmode.OpModeBase;
import org.firstinspires.ftc.teamcode.opmode.OpModeConstants;
import org.firstinspires.ftc.teamcode.subsystem.detector.RingDetector;
import org.firstinspires.ftc.teamcode.subsystem.detector.RingPipeline;
import org.firstinspires.ftc.teamcode.subsystem.drivetrain.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystem.drivetrain.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystem.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystem.shooter.Barrel;
import org.firstinspires.ftc.teamcode.subsystem.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.wobblegoal.Wobble;

@Autonomous
public class BlueDoubleWobble extends OpModeBase {

    private DriveConstraints baseConstraints = new DriveConstraints(50,30,0,Math.toRadians(180),Math.toRadians(180),0);

    private MecanumConstraints slowConstraints = new MecanumConstraints(baseConstraints, DriveConstants.TRACK_WIDTH);

    public static double TARGET_RPM = 4050;
    public static double TARGET_ANGLE = 9;
    public static double ANGLE_OFFSET = -4;
    public static double DELAY = 10;

    public static int TIME_BETWEEN_SHOTS = 600;

    private DriveTrain drive;
    private Shooter shooter;
    private Barrel barrel;
    private Intake intake;
    private Wobble wobble;
    private RingDetector detector;

    private RingPipeline.Rings rings = RingPipeline.Rings.FOUR;

    public static Pose2d START_POSE = new Pose2d(-63.0, 18.0, 0.0);
    public static Pose2d DETECT_POSE = new Pose2d(-48,20, Math.PI/4);
    public static Pose2d SHOOT_POSE = new Pose2d(
            -8,8,Math.atan2(OpModeConstants.BLUE_HIGH_GOAL.getY()-8,
            OpModeConstants.BLUE_HIGH_GOAL.getX()-(-8)) + Math.toRadians(ANGLE_OFFSET));
    public static Pose2d NO_RING_WOBBLE_POSE = new Pose2d(8,48, Math.PI/4);
    public static Pose2d ONE_RING_WOBBLE_POSE = new Pose2d(33,26,Math.PI/3);
    public static Pose2d FOUR_RING_WOBBLE_POSE = new Pose2d(52,43,Math.PI/4);

    public static Pose2d NO_RING_TURN_POSE = new Pose2d(18,30, Math.PI);
    public static Pose2d ONE_RING_TURN_POSE = new Pose2d(37,12,Math.PI);
    public static Pose2d FOUR_RING_TURN_POSE = new Pose2d(52,35,Math.PI);

    public static Pose2d FOUR_RING_HOME_WOBBLE_POSE = new Pose2d(-37.5,40.5,3 * Math.PI/4);
    public static Pose2d ONE_RING_HOME_WOBBLE_POSE = new Pose2d(-37.5,41.5,3 * Math.PI/4);
    public static Pose2d NO_RING_HOME_WOBBLE_POSE = new Pose2d(-37.5,41.5,3 * Math.PI/4);

    public static Pose2d FOUR_RING_WOBBLE_TWO_POSE = new Pose2d(40,46,0.0);
    public static Pose2d ONE_RING_WOBBLE_TWO_POSE = new Pose2d(18,26,0.0);
    public static Pose2d NO_RING_WOBBLE_TWO_POSE = new Pose2d(-8,46.5,0.0);

    public static Pose2d SECOND_TURN_POSE = new Pose2d(-37,34, 0.0);

    public static Pose2d PICK_UP_POSE = new Pose2d(-22.5,22,Math.PI/2 + Math.toRadians(5));

    public void runOpMode() throws InterruptedException {

        drive = new DriveTrain(hardwareMap);
        shooter = new Shooter(hardwareMap);
        barrel = new Barrel(hardwareMap);
        intake = new Intake(hardwareMap);
        wobble = new Wobble(hardwareMap);
        detector = new RingDetector(hardwareMap, "camera");

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
        Trajectory detectTraj = drive.trajectoryBuilder(START_POSE,0.0)
                .addTemporalMarker(0.1,()->{
                    shooter.setRpm(TARGET_RPM);
                })
                .splineToLinearHeading(DETECT_POSE,0.0)

                .build();

        Trajectory shootTraj = drive.trajectoryBuilder(DETECT_POSE, 0.0)
                .addTemporalMarker(0.1,()->{
                    shooter.setRpm(TARGET_RPM);
                })
                .splineToLinearHeading(SHOOT_POSE, 0.0)
                .build();

        Trajectory wobbleTraj4 = drive.trajectoryBuilder(SHOOT_POSE, SHOOT_POSE.getHeading() + Math.toRadians(ANGLE_OFFSET))
                .splineTo(new Vector2d(FOUR_RING_WOBBLE_POSE.getX(),FOUR_RING_WOBBLE_POSE.getY()), Math.PI/4)
                .addTemporalMarker(0.1,()->shooter.setPower(0))
                .addTemporalMarker(1.7,()->wobble.extend())
                .addTemporalMarker(2.1,()->wobble.open())
                .build();

        Trajectory wobbleTraj1 = drive.trajectoryBuilder(SHOOT_POSE,SHOOT_POSE.getHeading() + Math.toRadians(ANGLE_OFFSET))
                .splineToLinearHeading(ONE_RING_WOBBLE_POSE, Math.PI/2)
                .addTemporalMarker(0.1,()->shooter.setPower(0))
                .addTemporalMarker(1,()->wobble.extend())
                .build();

        Trajectory wobbleTraj0 = drive.trajectoryBuilder(SHOOT_POSE,SHOOT_POSE.getHeading() + Math.toRadians(ANGLE_OFFSET))
                .splineTo(NO_RING_WOBBLE_POSE.vec(),NO_RING_WOBBLE_POSE.getHeading())
                .addTemporalMarker(0.1,()->shooter.setPower(0))
                .addTemporalMarker(1,()->wobble.extend())
                .build();

        Trajectory pickUpTraj = drive.trajectoryBuilder(SHOOT_POSE, SHOOT_POSE.getHeading() + Math.toRadians(ANGLE_OFFSET))
                .splineToLinearHeading(PICK_UP_POSE,Math.PI/2)
                .build();

        Trajectory turnTraj0 = drive.trajectoryBuilder(NO_RING_WOBBLE_POSE, Math.PI)
                .splineToLinearHeading(NO_RING_TURN_POSE, -Math.PI/2)
                .addTemporalMarker(0.1,()->wobble.retract())
                .addTemporalMarker(0.2,()->wobble.open())
                .build();

        Trajectory turnTraj1 = drive.trajectoryBuilder(ONE_RING_WOBBLE_POSE, Math.PI)
                .splineToLinearHeading(ONE_RING_TURN_POSE, -Math.PI/2)
                .addTemporalMarker(0.1,()->wobble.retract())
                .addTemporalMarker(0.2,()->wobble.open())
                .build();

        Trajectory turnTraj4 = drive.trajectoryBuilder(FOUR_RING_WOBBLE_POSE, Math.PI)
                .lineToLinearHeading(FOUR_RING_TURN_POSE)
                .addTemporalMarker(0.1,()->wobble.retract())
                .addTemporalMarker(0.2,()->wobble.open())
                .build();

        Trajectory secondWobblePick0 = new TrajectoryBuilder(NO_RING_TURN_POSE,slowConstraints)
                .splineTo(new Vector2d(NO_RING_HOME_WOBBLE_POSE.getX(),NO_RING_HOME_WOBBLE_POSE.getY()),Math.PI*3/4)
                .addTemporalMarker(1,()->wobble.extend())
                .build();

        Trajectory secondWobblePick1 = new TrajectoryBuilder(ONE_RING_TURN_POSE,slowConstraints)
                .splineTo(new Vector2d(ONE_RING_HOME_WOBBLE_POSE.getX(), ONE_RING_HOME_WOBBLE_POSE.getY()),Math.PI*3/4)
                .addTemporalMarker(1.5 ,()->wobble.extend())
                .build();

        Trajectory secondWobblePick4 = drive.trajectoryBuilder(FOUR_RING_TURN_POSE)
                .splineTo(new Vector2d(FOUR_RING_HOME_WOBBLE_POSE.getX(),FOUR_RING_HOME_WOBBLE_POSE.getY()),Math.PI*3/4)
                .addTemporalMarker(1.7,()->wobble.extend())
                .build();

        Trajectory secondTurn4 = drive.trajectoryBuilder(FOUR_RING_HOME_WOBBLE_POSE)
                .lineToLinearHeading(SECOND_TURN_POSE)
                //.addTemporalMarker(0.1,()->wobble.retract())
                .build();

        Trajectory secondTurn1 = drive.trajectoryBuilder(ONE_RING_HOME_WOBBLE_POSE)
                .lineToLinearHeading(SECOND_TURN_POSE)
                .build();

        Trajectory secondTurn0 = drive.trajectoryBuilder(NO_RING_HOME_WOBBLE_POSE)
                .lineToLinearHeading(SECOND_TURN_POSE)
                .build();

        Trajectory secondDeliver4 = drive.trajectoryBuilder(SECOND_TURN_POSE)
                .splineTo(new Vector2d(FOUR_RING_WOBBLE_TWO_POSE.getX(),FOUR_RING_WOBBLE_TWO_POSE.getY()),FOUR_RING_WOBBLE_TWO_POSE.getHeading())
                .addTemporalMarker(2,()->wobble.open())
                .build();

        Trajectory secondDeliver1 = drive.trajectoryBuilder(SECOND_TURN_POSE)
                .splineTo(new Vector2d(ONE_RING_WOBBLE_TWO_POSE.getX(),ONE_RING_WOBBLE_TWO_POSE.getY()),ONE_RING_WOBBLE_TWO_POSE.getHeading())
                .build();

        Trajectory secondDeliver0 = drive.trajectoryBuilder(SECOND_TURN_POSE)
                .splineTo(new Vector2d(NO_RING_WOBBLE_TWO_POSE.getX(),NO_RING_WOBBLE_TWO_POSE.getY()),NO_RING_WOBBLE_TWO_POSE.getHeading())
                .build();

        Trajectory homeTraj4 = drive.trajectoryBuilder(FOUR_RING_WOBBLE_TWO_POSE, true)
                .lineToConstantHeading(new Vector2d(16,40))
                .addTemporalMarker(0.3,()->wobble.retract())
                .build();

        Trajectory homeTraj1 = drive.trajectoryBuilder(ONE_RING_WOBBLE_TWO_POSE, true)
                .lineToConstantHeading(new Vector2d(12,30))
                .addTemporalMarker(0.3,()->wobble.retract())
                .build();

        Trajectory homeTraj0 = drive.trajectoryBuilder(NO_RING_WOBBLE_TWO_POSE, Math.PI)
                .splineToConstantHeading(new Vector2d(8,20),0.0)
                .addTemporalMarker(0.3,()->wobble.retract())
                .build();

        while(!isStarted() && !isStopRequested()){
            //rings = detector.getRings();
        }
        waitForStart();
        if(isStopRequested()){
            return;
        }


        wobble.retract();

        drive.followTrajectoryAsync(detectTraj);
        while(drive.isBusy() && opModeIsActive()){
            drive.update();
            shooter.updatePIDCoeffs();
            rings = detector.getRings();
            telemetry.addData("Rings",rings);
            telemetry.update();
        }
        detector.close();

        drive.followTrajectoryAsync(shootTraj);
        while(drive.isBusy() && opModeIsActive()){
            drive.update();
            shooter.updatePIDCoeffs();
        }

        waitTimeMillis(200);

        for(int i = 0; i < 3; i++){
            barrel.extend();
            waitTimeMillis(TIME_BETWEEN_SHOTS/2);
            barrel.retract();
            waitTimeMillis(TIME_BETWEEN_SHOTS/2);
        }

        barrel.lower();

        switch(rings){
            case FOUR:
                drive.followTrajectoryAsync(pickUpTraj);
                while(!isStopRequested() && drive.isBusy()){ drive.update(); shooter.updatePIDCoeffs(); }

                drive.followTrajectoryAsync(
                        drive.trajectoryBuilder(drive.getPoseEstimate())
                                .forward(6)
                                .addTemporalMarker(0.1,()->intake.setPower(OpModeConstants.INTAKE_POWER))
                                .build()
                ); while(!isStopRequested() && drive.isBusy()){ drive.update(); shooter.updatePIDCoeffs(); }
                waitTimeMillis(300);
                intake.setPower(-1);
                waitTimeMillis(200);

                drive.followTrajectoryAsync(
                        drive.trajectoryBuilder(drive.getPoseEstimate())
                                .forward(5)
                                .addTemporalMarker(0.1,()->intake.setPower(OpModeConstants.INTAKE_POWER))
                                .build()
                ); while(!isStopRequested() && drive.isBusy()){ drive.update(); shooter.updatePIDCoeffs(); }
                waitTimeMillis(300);
                intake.setPower(-1);
                waitTimeMillis(200);

                drive.followTrajectoryAsync(
                        drive.trajectoryBuilder(drive.getPoseEstimate())
                                .forward(6)
                                .addTemporalMarker(0.1,()->intake.setPower(OpModeConstants.INTAKE_POWER))
                                .build()
                ); while(!isStopRequested() && drive.isBusy()){ drive.update(); shooter.updatePIDCoeffs(); }
                waitTimeMillis(300);
                intake.setPower(-1);
                waitTimeMillis(200);


                drive.followTrajectoryAsync(
                        drive.trajectoryBuilder(drive.getPoseEstimate(),true)
                                .addTemporalMarker(0.6, ()->{
                                    intake.setPower(-1);
                                    shooter.setRpm(4050);
                                    barrel.raise();
                                    barrel.retract();
                                })
                                .splineToLinearHeading(new Pose2d(
                                        SHOOT_POSE.getX(),
                                        SHOOT_POSE.getY(),
                                        SHOOT_POSE.getHeading() + Math.toRadians(1.5)
                                ),0.0)
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
                drive.followTrajectoryAsync(pickUpTraj); while(!isStopRequested() && drive.isBusy()){ drive.update(); shooter.updatePIDCoeffs();}

                intake.setPower(OpModeConstants.INTAKE_POWER);
                drive.followTrajectoryAsync(
                        drive.trajectoryBuilder(drive.getPoseEstimate())
                                .forward(11)
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
                                        SHOOT_POSE.getHeading() + Math.toRadians(2.5)
                                ),0.0)
                                .addTemporalMarker(0.1, ()->{
                                    shooter.setRpm(4050);
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

                while(!isStopRequested() && drive.isBusy()){ drive.update(); shooter.updatePIDCoeffs(); }

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

        barrel.lower();

        while(drive.isBusy() && opModeIsActive()) { drive.update(); shooter.updatePIDCoeffs(); }
        wobble.open();

        switch(rings){
            case ZERO:
                waitTimeMillis(500);
                wobble.retract();
                waitTimeMillis(500);
                drive.followTrajectoryAsync(turnTraj0);
                while(drive.isBusy() && opModeIsActive()) { drive.update(); shooter.updatePIDCoeffs(); }
                drive.followTrajectoryAsync(secondWobblePick0);
                while(drive.isBusy() && opModeIsActive()) { drive.update(); shooter.updatePIDCoeffs(); }
                wobble.close();
                waitTimeMillis(700);
                drive.followTrajectoryAsync(secondTurn0);
                while(drive.isBusy() && opModeIsActive()) { drive.update(); shooter.updatePIDCoeffs(); }
                drive.followTrajectoryAsync(secondDeliver0);
                while(drive.isBusy() && opModeIsActive()) { drive.update(); shooter.updatePIDCoeffs(); }
                wobble.open();
                waitTimeMillis(600);
                wobble.retract();
                waitTimeMillis(500);
                drive.followTrajectoryAsync(homeTraj0);
                while(drive.isBusy() && opModeIsActive()) { drive.update(); shooter.updatePIDCoeffs(); }
                break;
            case ONE:
                waitTimeMillis(500);
                wobble.retract();
                waitTimeMillis(500);
                drive.followTrajectoryAsync(turnTraj1);
                while(drive.isBusy() && opModeIsActive()) { drive.update(); shooter.updatePIDCoeffs(); }
                drive.followTrajectoryAsync(secondWobblePick1);
                while(drive.isBusy() && opModeIsActive()) { drive.update(); shooter.updatePIDCoeffs(); }
                wobble.close();
                waitTimeMillis(700);
                drive.followTrajectoryAsync(secondTurn1);
                while(drive.isBusy() && opModeIsActive()) { drive.update(); shooter.updatePIDCoeffs(); }
                drive.followTrajectoryAsync(secondDeliver1);
                while(drive.isBusy() && opModeIsActive()) { drive.update(); shooter.updatePIDCoeffs(); }
                wobble.open();
                waitTimeMillis(600);
                drive.followTrajectoryAsync(homeTraj1);
                while(drive.isBusy() && opModeIsActive()) { drive.update(); shooter.updatePIDCoeffs(); }
                break;
            case FOUR:
                drive.followTrajectoryAsync(turnTraj4);
                while(drive.isBusy() && opModeIsActive()) { drive.update(); shooter.updatePIDCoeffs(); }
                drive.followTrajectoryAsync(secondWobblePick4);
                while(drive.isBusy() && opModeIsActive()) { drive.update(); shooter.updatePIDCoeffs(); }
                wobble.close();
                waitTimeMillis(700);
                drive.followTrajectoryAsync(secondTurn4);
                while(drive.isBusy() && opModeIsActive()) { drive.update(); shooter.updatePIDCoeffs(); }
                drive.followTrajectoryAsync(secondDeliver4);
                while(drive.isBusy() && opModeIsActive()) { drive.update(); shooter.updatePIDCoeffs(); }
                drive.followTrajectoryAsync(homeTraj4);
                while(drive.isBusy() && opModeIsActive()) { drive.update(); shooter.updatePIDCoeffs(); }
                break;
        }
        OpModeConstants.AutoEndPose = drive.getPoseEstimate();
    }



}
