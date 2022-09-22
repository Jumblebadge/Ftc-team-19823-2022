package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import org.firstinspires.ftc.teamcode.pipelines.greyPipe;
import org.firstinspires.ftc.teamcode.pipelines.aprilTagDetectPipe;

import java.util.ArrayList;


@Config
@TeleOp(name="visionTests", group="Linear Opmode")
public class visionTests extends LinearOpMode {

    OpenCvWebcam webcam;

    int side1 = 1;
    int side2 = 2;
    int side3 = 3;

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    double tagsize = 0.166;

    aprilTagDetectPipe pipeline = new aprilTagDetectPipe(tagsize,fx,fy,cx,cy);

    AprilTagDetection detectedTag = null;

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
        webcam.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);

        webcam.setPipeline(pipeline);

        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

            @Override
            public void onOpened()
            {
                webcam.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        telemetry.addLine("Waiting for start");
        FtcDashboard.getInstance().startCameraStream(webcam, 60);

        while (!isStarted() && !isStopRequested()) {

            ArrayList<AprilTagDetection> detecteds = pipeline.getLatestDetections();

            if(detecteds.size() != 0) {
                boolean tagFound = false;

                for(AprilTagDetection tag : detecteds) {
                    if(tag.id == side1 || tag.id == side2 || tag.id == side3) {
                        detectedTag = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound) {
                    telemetry.addLine("can see tag \n");
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
                    telemetry.addLine("never seen tag");
                }
                else
                {
                    telemetry.addLine("\nlast seen tag");
                    tagToTelemetry(detectedTag);
                }

            }

            telemetry.update();
            sleep(20);
        }


        webcam.closeCameraDevice();
        //auto starting

        if(detectedTag != null)
        {
            telemetry.addLine("tag:\n");
            tagToTelemetry(detectedTag);
        }
        else
        {
            telemetry.addLine("never seen tag");
        }
        telemetry.update();

        //auton code
        if(detectedTag == null /*|| detectedTag.id == default*/) {
           //go to default position after auton code
        }
        else if(detectedTag.id == side2){

        }
        else if(detectedTag.id == side3){

        }



        while (opModeIsActive()) {

            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("tag",detectedTag.id);
            telemetry.update();

        }
    }
    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
    }
}

