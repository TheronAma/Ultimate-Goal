package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.opmode.OpModeConstants;
import org.firstinspires.ftc.teamcode.subsystem.drivetrain.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystem.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystem.shooter.Barrel;
import org.firstinspires.ftc.teamcode.subsystem.shooter.Const;
import org.firstinspires.ftc.teamcode.subsystem.shooter.Shooter;

@Config
public class TestAuto extends LinearOpMode {
    public static Pose2d START_POSE = new Pose2d(24,-62,Math.PI/4);
    public static Vector2d SHOOT_POS = new Vector2d(14,-40);

    private ElapsedTime waitTimer;
    private DriveTrain drive;
    private Shooter shooter;
    private Barrel barrel;
    private Intake intake;

    public void runOpMode(){
        waitTimer = new ElapsedTime();
        drive = new DriveTrain(hardwareMap);
        shooter = new Shooter(hardwareMap);
        barrel = new Barrel(hardwareMap);
        intake = new Intake(hardwareMap);

        /*
          initialization
         */
        shooter.setAngle(Const.FLAP_AUTO_ANGLE);
        barrel.raise();
        barrel.retract();

        drive.setPoseEstimate(START_POSE);

        /*
          vision
         */

        waitForStart();
        //Main op mode starts here
        double angle = Math.atan2(
                OpModeConstants.RED_HIGH_GOAL.getY() - SHOOT_POS.getY(),
                OpModeConstants.RED_HIGH_GOAL.getX() - SHOOT_POS.getX()
        );

        Trajectory moveToShotTraj = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineTo(SHOOT_POS,angle)
                .addTemporalMarker(0.2,()->{
                    //intake.setPower(0.8);
                    //shooter.setRpm(Const.SHOOTER_REGULAR_RPM);
                })
                .build();

        drive.followTrajectoryAsync(moveToShotTraj);
        while(drive.isBusy() && opModeIsActive()){
            drive.update();
        }



    }

    /**
     * idles for a certain amount of time
     * @param time in milliseconds
     */

    public void waitTimeMillis(double time){
        waitTimer.reset();
        while(waitTimer.milliseconds() <= time && opModeIsActive()){
            drive.update();
        }
    }
}
