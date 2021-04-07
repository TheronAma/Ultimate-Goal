package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystem.detector.HighGoalDetector;
import org.firstinspires.ftc.teamcode.subsystem.detector.RingDetector;
import org.firstinspires.ftc.teamcode.subsystem.detector.RingDetector;
import org.firstinspires.ftc.teamcode.subsystem.detector.UGBasicHighGoalPipeline;

@Autonomous
public class CameraTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException{

        HighGoalDetector detector = new HighGoalDetector(hardwareMap,"camera");
        detector.init();
        waitForStart();

        while(opModeIsActive()){
            if(detector.pipeline.isBlueVisible()){
                telemetry.addData("Blue center position",detector.pipeline.getCenterofRect(detector.pipeline.getBlueRect()));
            }
            //telemetry.addData("Position", detector.getRings());
            telemetry.update();
        }
        detector.close();

    }
}
