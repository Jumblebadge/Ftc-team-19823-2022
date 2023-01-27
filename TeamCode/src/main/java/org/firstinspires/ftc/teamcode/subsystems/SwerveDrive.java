package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.maths.PIDcontroller;
import org.firstinspires.ftc.teamcode.maths.mathsOperations;
import org.firstinspires.ftc.teamcode.maths.swerveKinematics;

import java.util.List;

public class SwerveDrive {

    final private BNO055IMU IMU;
    final private DcMotorEx mod1m1,mod1m2,mod2m1,mod2m2,mod3m1,mod3m2;
    final private AnalogInput mod1E,mod2E,mod3E;
    final private List<LynxModule> allHubs;
    final private Telemetry telemetry;
    final private VoltageSensor vSensor;
    final private boolean eff;
    private double Kp,Kd,Ki,Kf;
    private double module1Adjust = 0, module2Adjust = 0, module3Adjust = 0;
    PIDcontroller mod1PID = new PIDcontroller(0.2,0.003,0.01,0.75);
    PIDcontroller mod2PID = new PIDcontroller(0.2,0.003,0.01,1);
    PIDcontroller mod3PID = new PIDcontroller(0.2,0.003,0.01,1.5);
    swerveKinematics swavemath = new swerveKinematics();

    public SwerveDrive(Telemetry telemetry, DcMotorEx mod1m1, DcMotorEx mod1m2, DcMotorEx mod2m1, DcMotorEx mod2m2, DcMotorEx mod3m1, DcMotorEx mod3m2, AnalogInput mod1E, AnalogInput mod2E, AnalogInput mod3E, BNO055IMU IMU, List<LynxModule> allHubs, VoltageSensor vSensor, boolean eff){
        this.mod1m1 = mod1m1;
        this.mod1m2 = mod1m2;
        this.mod2m1 = mod2m1;
        this.mod2m2 = mod2m2;
        this.mod3m1 = mod3m1;
        this.mod3m2 = mod3m2;
        this.mod1E = mod1E;
        this.mod2E = mod2E;
        this.mod3E = mod3E;
        this.IMU = IMU;
        this.allHubs = allHubs;
        this.telemetry = telemetry;
        this.vSensor = vSensor;
        this.eff = eff;
    }

    boolean mod1wrapped = true, mod2wrapped = false, mod3wrapped = false;
    double mod1lastpos = 0, mod2lastpos = 0, mod3lastpos = 0;

    Orientation angles;

    double mod1reference1 = 0;
    double mod2reference1 = 0;
    double mod3reference1 = 0;

    public void driveOut(double x, double y, double rot){

        double voltageConstant = 12/vSensor.getVoltage();

        //Turn our MA3 absolute encoder signals from volts to degrees
        double mod1P = mod1E.getVoltage() * 74.16;
        double mod2P = mod2E.getVoltage() * 74.16;
        double mod3P = mod3E.getVoltage() * 74.16;

        //Update heading of robot
        angles   = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double heading = angles.firstAngle*-1;

        //Retrieve the angles and powers for all of our wheels from the swerve kinematics
        double[] output = swavemath.calculate(y*voltageConstant,-x*voltageConstant,rot*voltageConstant,heading,true);
        double mod1power=-output[0];
        double mod3power=output[1];
        double mod2power=output[2];

        if (y!=0||x!=0||rot!=0){
            mod1reference1=output[3];
            mod3reference1=output[5];
            mod2reference1=-output[4];
        }

        double mod1reference=mod1reference1;
        double mod3reference=mod3reference1;
        double mod2reference=-mod2reference1;

        //Subtract our tuning values to account for any encoder drift
        mod3P -= module3Adjust;
        mod2P -= module2Adjust;
        mod1P -= module1Adjust;

        //Anglewrap all the angles so that the module turns both ways
        mod1P = mathsOperations.angleWrap(mod1P);
        mod2P = mathsOperations.angleWrap(mod2P);
        mod3P = mathsOperations.angleWrap(mod3P);

        mod1reference = mathsOperations.angleWrap(mod1reference);
        mod2reference = mathsOperations.angleWrap(mod2reference);
        mod3reference = mathsOperations.angleWrap(mod3reference);

        //Make sure that a module never turns more than 90 degrees
        double[] mod1efvalues = mathsOperations.efficientTurn(mod1reference,mod1P,mod1power);

        double[] mod2efvalues = mathsOperations.efficientTurn(mod2reference,mod2P,mod2power);

        double[] mod3efvalues = mathsOperations.efficientTurn(mod3reference,mod3P,mod3power);

        if (eff){
            mod1reference=mod1efvalues[0];
            mod1power=mod1efvalues[1];
            mod2reference=mod2efvalues[0];
            mod2power=mod2efvalues[1];
            mod3reference=mod3efvalues[0];
            mod3power=mod3efvalues[1];
        }

        //change coax values into diffy values, from pid and power
        double[] mod1values = mathsOperations.diffyConvert(mod1PID.out(AngleUnit.normalizeDegrees(mod1reference-mod1P)),mod1power);
        mod1m1.setPower(mod1values[0]);
        mod1m2.setPower(mod1values[1]);
        double[] mod2values = mathsOperations.diffyConvert(-mod2PID.out(AngleUnit.normalizeDegrees(mod2reference-mod2P)),mod2power);
        mod2m1.setPower(mod2values[0]);
        mod2m2.setPower(mod2values[1]);
        double[] mod3values = mathsOperations.diffyConvert(mod3PID.out(AngleUnit.normalizeDegrees(mod3reference-mod3P)),-mod3power);
        mod3m1.setPower(mod3values[0]);
        mod3m2.setPower(mod3values[1]);

        telemetry.addData("mod1reference",mod1reference);
        telemetry.addData("mod2reference",mod2reference);
        telemetry.addData("mod3reference",mod3reference);

        telemetry.addData("mod1P",mod1P);
        telemetry.addData("mod2P",mod2P);
        telemetry.addData("mod3P",mod3P);

        telemetry.addData("mod3power",mod3power);
        telemetry.addData("mod2power",mod2power);
        telemetry.addData("mod1power",mod1power);
    }
    public void setPIDCoeffs(double Kp, double Kd,double Ki, double Kf){
        this.Kp = Kp;
        this.Kd = Kd;
        this.Ki = Ki;
        this.Kf = Kf;
    }
    public void setModuleAdjustments(double module1Adjust, double module2Adjust, double module3Adjust){
        this.module1Adjust=module1Adjust;
        this.module2Adjust=module2Adjust;
        this.module3Adjust=module3Adjust;
    }
}
