package org.firstinspires.ftc.teamcode.subsystem.drivetrain;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.kinematics.MecanumKinematics;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/*
This is our base DriveTrain class
A lot of this code is adapted from
the roadrunner quickstart, with some modifications
 */

public class DriveTrain extends MecanumDrive {

    /*
        These are the drive constants and constraints.
        These will be used for calculations required in order
        to follow trajectories, and for feedforward control.
     */

    public static final double TICKS_PER_REV = 537.6;
    public static final double MAX_RPM = 312;

    public static final boolean RUN_USING_ENCODER = true;

    public static final double WHEEL_RADIUS = 2;
    public static final double GEAR_RATIO = 1;
    public static final double MAX_VELOCITY = 312 * GEAR_RATIO * WHEEL_RADIUS * 2 * Math.PI / 60.0;
    public static double TRACK_WIDTH = 15;

    public static double kV = 1.0 / (MAX_VELOCITY); //ratio between motor power and velocity
    public static double kRotate = 0; //ratio between motor power and rotational acceleration
    public static double kA = 0; //ratio of motor power to acceleration
    public static double kStatic = 0;//amount of power required to overcome static friction
    public static double LATERAL_MULTIPLIER = 1;//ratio of power required to make horizontal movements

    public static DriveConstraints BASE_CONSTRAINTS = new DriveConstraints(
            30.0, 30.0, 0.0,
            Math.toRadians(180.0), Math.toRadians(180.0), 0.0
    );

    public static PIDCoefficients translationalPID = new PIDCoefficients(0,0,0);
    public static PIDCoefficients headingPID = new PIDCoefficients(0,0,0);

    private DcMotorEx leftFront, leftRear, rightRear, rightFront;

    private FtcDashboard dashboard;
    private NanoClock clock;

    public enum Mode {
        FOLLOW_TRAJECTORY,
        TELE_OP,
        GO_TO_POSE,
        TURN
    }

    private Mode mode; //mode the robot is in

    private PIDFController turnController; //controller for turn mode
    private MotionProfile turnProfile;
    private double turnStart;

    private DriveConstraints constraints; //constraints for the robot
    private TrajectoryFollower follower;

    private List<Pose2d> poseHistory;

    private List<DcMotorEx> motors;
    private BNO055IMU imu;

    private Pose2d lastPoseOnTurn;

    private Pose2d targetPose;
    public static double kPtrans = 0.1;
    public static double kPHeading = 0.1;

    public DriveTrain(HardwareMap hwMap){
        super(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);

        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);

        mode = Mode.TELE_OP;

        clock = NanoClock.system();

        turnController = new PIDFController(headingPID);
        turnController.setInputBounds(0, 2 * Math.PI);

        constraints = new MecanumConstraints(BASE_CONSTRAINTS, TRACK_WIDTH);
        follower = new HolonomicPIDVAFollower(translationalPID, translationalPID, headingPID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);

        poseHistory = new ArrayList<>();

        for (LynxModule module : hwMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        imu = hwMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        leftFront = hwMap.get(DcMotorEx.class, "leftFront");
        leftRear = hwMap.get(DcMotorEx.class, "leftBack");
        rightRear = hwMap.get(DcMotorEx.class, "rightBack");
        rightFront = hwMap.get(DcMotorEx.class, "rightFront");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        setLocalizer(new Localizer(hwMap));

        //TODO Reverse motors when we actually get robot

    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, constraints);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, constraints);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, constraints);
    }

    public void turnAsync(double angle) {
        double heading = getPoseEstimate().getHeading();

        lastPoseOnTurn = getPoseEstimate();

        turnProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(heading, 0, 0, 0),
                new MotionState(heading + angle, 0, 0, 0),
                constraints.maxAngVel,
                constraints.maxAngAccel,
                constraints.maxAngJerk
        );

        turnStart = clock.seconds();
        mode = Mode.TURN;
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        follower.followTrajectory(trajectory);
        mode = Mode.FOLLOW_TRAJECTORY;
    }

    public Pose2d getLastError() {
        switch (mode) {
            case FOLLOW_TRAJECTORY:
                return follower.getLastError();
            case TURN:
                return new Pose2d(0, 0, turnController.getLastError());
            default:
                return new Pose2d();
        }

    }

    public void setTargetPose(Pose2d target){
        targetPose = target;
        mode = Mode.GO_TO_POSE;
    }

    public void update() {
        updatePoseEstimate();
        Pose2d currentPose = getPoseEstimate();
        Pose2d lastError = getLastError();

        poseHistory.add(currentPose);

        switch(mode){
            case TURN: {
                double t = clock.seconds() - turnStart;

                MotionState targetState = turnProfile.get(t);

                turnController.setTargetPosition(targetState.getX());

                double correction = turnController.update(currentPose.getHeading());

                double targetOmega = targetState.getV();
                double targetAlpha = targetState.getA();
                setDriveSignal(new DriveSignal(new Pose2d(
                        0, 0, targetOmega + correction
                ), new Pose2d(
                        0, 0, targetAlpha
                )));

                Pose2d newPose = lastPoseOnTurn.copy(lastPoseOnTurn.getX(), lastPoseOnTurn.getY(), targetState.getX());
                if (t >= turnProfile.duration()) {
                    mode = Mode.TELE_OP;
                    setDriveSignal(new DriveSignal());
                }
                break;
            }
            case FOLLOW_TRAJECTORY: {
                setDriveSignal(follower.update(currentPose));

                if (!follower.isFollowing()) {
                    mode = Mode.TELE_OP;
                    setDriveSignal(new DriveSignal());
                }
                break;
            }
            case GO_TO_POSE:{
                Pose2d error = targetPose.minus(getPoseEstimate());
                setDriveSignal(new DriveSignal(
                        new Pose2d(
                                error.getX() * kPtrans,
                                error.getY() * kPtrans,
                                error.getHeading() * kPHeading
                        ),
                        new Pose2d()
                ));
            }
        }
    }

    public boolean isBusy() {
        return mode != Mode.TELE_OP;
    }

    public void setRunMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public PIDCoefficients getMotorPIDCoefficients(DcMotor.RunMode runMode) {
        PIDFCoefficients coefficients = leftFront.getPIDFCoefficients(runMode);
        return new PIDCoefficients(coefficients.p, coefficients.i, coefficients.d);
    }

    public void setPIDCoefficients(DcMotor.RunMode runMode, PIDCoefficients coefficients) {
        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, new PIDFCoefficients(
                    coefficients.kP, coefficients.kI, coefficients.kD, 32767 * 60.0 / (MAX_RPM * TICKS_PER_REV)
            ));
        }
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelPositions.add(encoderTicksToInches(motor.getCurrentPosition()));
        }
        return wheelPositions;
    }

    public static double encoderTicksToInches(double ticks){
        return ticks * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / TICKS_PER_REV;
    }

    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelVelocities.add(encoderTicksToInches(motor.getVelocity()));
        }
        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }

    @Override
    public void setDriveSignal(DriveSignal driveSignal) {
        List<Double> velocities = MecanumKinematics.robotToWheelVelocities(
                driveSignal.getVel(), TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);
        List<Double> accelerations = MecanumKinematics.robotToWheelAccelerations(
                driveSignal.getAccel(), TRACK_WIDTH*kRotate/kA, TRACK_WIDTH, LATERAL_MULTIPLIER);
        List<Double> powers = Kinematics.calculateMotorFeedforward(velocities, accelerations, kV, kA, kStatic);
        setMotorPowers(powers.get(0), powers.get(1), powers.get(2), powers.get(3));
    }

    @Override
    public double getRawExternalHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

}
