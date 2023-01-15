package org.firstinspires.ftc.teamcode.subs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.roadrunner.profile.*;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.maths.controlLoopMath;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.wrappers.myElapsedTime;
import org.opencv.core.Point;

public class goToPoint {
    private final driveSwerve driver;
    private final Telemetry telemetry;
    private final FtcDashboard dashboard;
    private final myElapsedTime profileTime = new myElapsedTime();
    private final controlLoopMath headingPID = new controlLoopMath(6,0,0,0);
    private final controlLoopMath xPID = new controlLoopMath(2.1,0,0.15,0);
    private final controlLoopMath yPID = new controlLoopMath(2.1,0,0.15,0);
    private MotionProfile profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(0, 0, 0), new MotionState(1, 0, 0), 2, 3,4);

    public goToPoint(driveSwerve driver, Telemetry telemetry, FtcDashboard dashboard){
        this.driver=driver;
        this.telemetry = telemetry;
        this.dashboard = dashboard;
    }

    //TODO make this work for auto: make the if a while, update the current pose
    public void driveToPoint(Pose2d pose,Pose2d desiredPose,Pose2d startPose,boolean update){
        double distanceNow = Math.abs(Math.hypot(desiredPose.getX()-pose.getX(),desiredPose.getY()-pose.getY()));
        double distanceAtStart = Math.abs(Math.hypot(desiredPose.getX()-startPose.getX(),desiredPose.getY()-startPose.getY()));
        double distance = distanceAtStart-distanceNow;
        double angleToEndPoint = Math.atan2(desiredPose.getY()-startPose.getY(),desiredPose.getX()-startPose.getX());
        if(update){
            profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(0, 0, 0), new MotionState(distanceAtStart, 0, 0), 20, 20, 20);
            profileTime.reset();
        }
        MotionState state = profile.get(profileTime.seconds());
        double stateOut = state.getX();
        Point statePoint = new Point(startPose.getX()+(stateOut *Math.cos(angleToEndPoint)),startPose.getY()+(stateOut *Math.sin(angleToEndPoint)));
        double distanceToState = Math.abs(Math.hypot(statePoint.x-startPose.getX(),statePoint.y-startPose.getY()));
        double xOut = xPID.PIDout(distanceToState * Math.cos(angleToEndPoint) - (pose.getX() - startPose.getX()));
        double yOut = yPID.PIDout(distanceToState * Math.sin(angleToEndPoint) - (pose.getY() - startPose.getY()));
        double headingOut = headingPID.PIDout(AngleUnit.normalizeRadians(desiredPose.getHeading()-pose.getHeading()));
        driver.driveOut(-yOut,-xOut,-headingOut);
        drawField(pose,desiredPose,startPose,dashboard);

        telemetry.addData("distance: ",distance);
        telemetry.addData("timer,",profileTime.seconds());
        telemetry.addData("distanceAtStart: ",distanceAtStart);
        telemetry.addData("distanceToProfile",distanceToState);
        telemetry.addData("distanceNow: ",distanceNow);
        telemetry.addData("angleToEndPoint: ", angleToEndPoint);
        telemetry.addData("stateOut: ", stateOut);
        telemetry.addData("xOut: ", xOut);
        telemetry.addData("yOut: ", yOut);
        telemetry.addData("statepointX",statePoint.x);
        telemetry.addData("statepointY",statePoint.y);
        telemetry.addData("xerror",distanceToState*Math.cos(angleToEndPoint)-(pose.getX()-startPose.getX()));
        telemetry.addData("yerror",distanceToState*Math.sin(angleToEndPoint)-(pose.getY()-startPose.getY()));
    }

    /**
    public void setPIDCoeffs(double Kp, double Kd,double Ki, double Kf){
        this.Kp = Kp;
        this.Kd = Kd;
        this.Ki = Ki;
        this.Kf = Kf;
    }

    public void setHeadingPIDcoeffs(double Kp,double Kd, double Ki){
        this.headingKp = Kp;
        this.headingKd = Kd;
        this.headingKi = Ki;
    }

    public void setProfileConstraints(double maxVel, double maxAccel, double maxJerk){
        this.maxVel = maxVel;
        this.maxAccel = maxAccel;
        this.maxJerk = maxJerk;
    }
     **/

    public static void drawRobot(Canvas canvas, Pose2d pose) {
        canvas.strokeCircle(pose.getX(), pose.getY(), 9);
        Vector2d v = pose.headingVec().times(9);
        double x1 = pose.getX() + v.getX() / 2, y1 = pose.getY() + v.getY() / 2;
        double x2 = pose.getX() + v.getX(), y2 = pose.getY() + v.getY();
        canvas.strokeLine(x1, y1, x2, y2);
    }

    public void drawField(Pose2d pose, Pose2d desiredPose, Pose2d startPose, FtcDashboard dash){
        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        fieldOverlay.setStrokeWidth(1);
        fieldOverlay.setStroke("#3F51B5");
        drawRobot(fieldOverlay, pose);
        fieldOverlay.setStroke("#51B53F");
        drawRobot(fieldOverlay,desiredPose);
        fieldOverlay.setStroke("#B53F51");
        drawRobot(fieldOverlay,startPose);

        packet.put("x", pose.getX());
        packet.put("y", pose.getY());
        packet.put("heading (deg)", Math.toDegrees(pose.getHeading()));

        dash.sendTelemetryPacket(packet);

    }
}
