package org.firstinspires.ftc.teamcode.pipelines;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

public class cameraActivity {

    protected OpenCvWebcam webcam;
    HardwareMap hardwareMap;
    Telemetry telemetry;
    private final double fx = 578.272;
    private final double fy = 578.272;
    private final double cx = 402.145;
    private final double cy = 221.506;
    private final double tagsize = 0.2;
    private AprilTagDetection detectedTag = null;

    aprilTagDetectPipe pipeline = new aprilTagDetectPipe(tagsize,fx,fy,cx,cy);

    public cameraActivity(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap=hardwareMap;
        this.telemetry=telemetry;
    }

    public void initCamera(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
        webcam.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);

        webcam.setPipeline(pipeline);

        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

            @Override
            public void onOpened() {
                webcam.startStreaming(800, 600, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        FtcDashboard.getInstance().startCameraStream(webcam, 60);

    }

    public void detectTags(){
        ArrayList<AprilTagDetection> detecteds = pipeline.getLatestDetections();

        if(detecteds.size() != 0) {
            boolean tagFound = false;

            for(AprilTagDetection tag : detecteds) {
                if(tag.id == 1 || tag.id == 2 || tag.id == 3) {
                    detectedTag = tag;
                    tagFound = true;
                    break;
                }
            }

            if(tagFound) {
                telemetry.addLine("cann see tag \n");
                tagToTelemetry(detectedTag);
            }
            else
            {
                telemetry.addLine("cant see tag");

                if(detectedTag == null)
                {
                    telemetry.addLine("never seen tag");
                }
                else
                {
                    telemetry.addLine("\nlast seen tag");
                    tagToTelemetry(detectedTag);
                }
            }

        }
        else
        {
            telemetry.addLine("cant see tag");

            if(detectedTag == null)
            {
                telemetry.addLine("neverr seen tag");
            }
            else
            {
                telemetry.addLine("\nlast seen tag");
                tagToTelemetry(detectedTag);
            }

        }

        telemetry.update();
        //TODO thread.sleep(20)
    }

    public void closeCamera(){
        webcam.closeCameraDevice();
    }

    public AprilTagDetection sideDetected(){
        return detectedTag;
    }

    public void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
    }
}
