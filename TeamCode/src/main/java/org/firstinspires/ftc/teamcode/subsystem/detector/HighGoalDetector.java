package org.firstinspires.ftc.teamcode.subsystem.detector;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
public class HighGoalDetector {
    private OpenCvCamera camera;
    private String cameraName;
    private HardwareMap hardwareMap;
    private Servo servo;
    public UGBasicHighGoalPipeline pipeline;

    public static double RAISE_POS = 0.4;
    public static double LOWER_POS = 0;

    public HighGoalDetector(HardwareMap hardwareMap, String cameraName){
        this.hardwareMap = hardwareMap;
        this.cameraName = cameraName;

        pipeline = new UGBasicHighGoalPipeline();
    }

    public void init(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, cameraName), cameraMonitorViewId);
        servo = hardwareMap.get(Servo.class,"camServo");


        camera.setPipeline(pipeline = new UGBasicHighGoalPipeline());
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);
            }
        });
    }

    public void close(){
        camera.closeCameraDevice();
    }

    public void pause(){
        camera.pauseViewport();
    }

    public void raise(){
        servo.setPosition(RAISE_POS);
    }

    public void lower(){
        servo.setPosition(LOWER_POS);
    }

}
