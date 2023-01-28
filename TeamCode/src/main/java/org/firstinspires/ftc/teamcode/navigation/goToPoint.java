package org.firstinspires.ftc.teamcode.navigation;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.profile.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.maths.PIDcontroller;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.SwerveDrive;
import org.firstinspires.ftc.teamcode.utility.myElapsedTime;
import org.opencv.core.Point;

public class goToPoint {
    private final SwerveDrive driver;
    private final Telemetry telemetry;
    private final FtcDashboard dashboard;
    private boolean isDone;
    private final myElapsedTime profileTime = new myElapsedTime();
    private final PIDcontroller headingPID = new PIDcontroller(6,0,0,0);
    private final PIDcontroller xPID = new PIDcontroller(2.1,0,0.15,0);
    private final PIDcontroller yPID = new PIDcontroller(2.1,0,0.15,0);
    private MotionProfile profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(0, 0, 0), new MotionState(1, 0, 0), 2, 3,4);

    public goToPoint(SwerveDrive driver, Telemetry telemetry, FtcDashboard dashboard){
        this.driver=driver;
        this.telemetry = telemetry;
        this.dashboard = dashboard;
    }

    public void driveToPoint(Pose2d pose,Pose2d desiredPose,Pose2d startPose,boolean update){
        double distanceNow = Math.abs(Math.hypot(desiredPose.getX()-pose.getX(),desiredPose.getY()-pose.getY()));
        double distanceAtStart = Math.abs(Math.hypot(desiredPose.getX()-startPose.getX(),desiredPose.getY()-startPose.getY()));
        //distance starts at 0 and goes to the distance from start as the robot gets closer
        double distance = distanceAtStart-distanceNow;
        //angle from start to finish
        double angleToEndPoint = Math.atan2(desiredPose.getY()-startPose.getY(),desiredPose.getX()-startPose.getX());
        //if the target position has changed, then create a new motion profile and reset the profile timer
        if(update){
            profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(0, 0, 0), new MotionState(distanceAtStart, 0, 0), 20, 20, 20);
            profileTime.reset();
        }
        MotionState state = profile.get(profileTime.seconds());
        double stateOut = state.getX();
        //create a point from the motion profile output, which is normally just a distance value
        Point statePoint = new Point(startPose.getX()+(stateOut *Math.cos(angleToEndPoint)),startPose.getY()+(stateOut *Math.sin(angleToEndPoint)));
        //distance from current position to the target point, determined by the motion profile
        double distanceToState = Math.abs(Math.hypot(statePoint.x-startPose.getX(),statePoint.y-startPose.getY()));
        //use pid on x and y position to move towards the target point determined by the state
        //distanceToState * sin or cos of the angle to end point will give the x or y component of the distance vector, which ensures that x and y arrive at the same time. pose-startpos is the current state
        double xOut = xPID.pidOut(distanceToState * Math.cos(angleToEndPoint) - (pose.getX() - startPose.getX()));
        double yOut = yPID.pidOut(distanceToState * Math.sin(angleToEndPoint) - (pose.getY() - startPose.getY()));
        double headingOut = headingPID.pidOut(AngleUnit.normalizeRadians(desiredPose.getHeading()-pose.getHeading()));
        //feed the pid output into swerve kinematics and draw the robot on FTCdash field
        driver.driveOut(-yOut,-xOut,-headingOut);
        drawField(pose,desiredPose,startPose,dashboard);

        isDone = distanceNow < 1;

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

    public boolean isDone(){
        return isDone;
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

    //draw the robot on the FTCdash field (copied from the roadrunner quickstart)
    public static void drawRobot(Canvas canvas, Pose2d pose) {
        canvas.strokeCircle(pose.getX(), pose.getY(), 9);
        Vector2d v = pose.headingVec().times(9);
        double x1 = pose.getX() + v.getX() / 2, y1 = pose.getY() + v.getY() / 2;
        double x2 = pose.getX() + v.getX(), y2 = pose.getY() + v.getY();
        canvas.strokeLine(x1, y1, x2, y2);
    }

    //draw all the robots on the field and send to the dashboard
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
