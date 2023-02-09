package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.maths.PIDcontroller;
import org.firstinspires.ftc.teamcode.maths.mathsOperations;
import org.firstinspires.ftc.teamcode.maths.swerveKinematics;

public class SwerveDrive {

    final private IMU imu;
    final private DcMotorEx mod1m1,mod1m2,mod2m1,mod2m2,mod3m1,mod3m2;
    final private AnalogInput mod1E,mod2E,mod3E;
    final private Telemetry telemetry;
    final private boolean eff;
    private double Kp,Kd,Ki,Kf;
    private double module1Adjust = 0, module2Adjust = 0, module3Adjust = 0;
    PIDcontroller mod1PID = new PIDcontroller(0.1,0.001,0,0.1);
    PIDcontroller mod2PID = new PIDcontroller(0.1,0.001,0,0.1);
    PIDcontroller mod3PID = new PIDcontroller(0.1,0.001,0,0.1);
    swerveKinematics swavemath = new swerveKinematics();

    public SwerveDrive(Telemetry telemetry, IMU imu, HardwareMap hardwareMap, boolean eff){
        mod1m1 = hardwareMap.get(DcMotorEx.class,"mod1m1");
        mod1m2 = hardwareMap.get(DcMotorEx.class,"mod1m2");
        mod2m1 = hardwareMap.get(DcMotorEx.class,"mod2m1");
        mod2m2 = hardwareMap.get(DcMotorEx.class,"mod2m2");
        mod3m1 = hardwareMap.get(DcMotorEx.class,"mod3m1");
        mod3m2 = hardwareMap.get(DcMotorEx.class,"mod3m2");
        mod1E = hardwareMap.get(AnalogInput.class, "mod1E");
        mod2E = hardwareMap.get(AnalogInput.class, "mod2E");
        mod3E = hardwareMap.get(AnalogInput.class, "mod3E");

        mod2m2.setDirection(DcMotorSimple.Direction.REVERSE);
        mod3m2.setDirection(DcMotorSimple.Direction.REVERSE);
        this.imu = imu;
        this.telemetry = telemetry;
        this.eff = eff;
    }

    double mod1reference = 0;
    double mod2reference = 0;
    double mod3reference = 0;

    public void drive(double x, double y, double rot){

        //mod3PID.setPIDCoeffs(Kp,Kd,Ki,Kf);

        //Turn our MA3 absolute encoder signals from volts to degrees
        double mod1P = mod1E.getVoltage() * 74.16;
        double mod2P = mod2E.getVoltage() * 74.16;
        double mod3P = mod3E.getVoltage() * 74.16;

        //Update heading of robot
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        double heading = angles.getYaw(AngleUnit.DEGREES)*-1;

        //Retrieve the angle and power for each module
        double[] output = swavemath.calculate(y,-x,-rot,heading,true);
        double mod1power = output[0];
        double mod3power = output[1];
        double mod2power = output[2];

        //keep previous module heading if joystick not being used
        if (y != 0 || x != 0 || rot != 0){
            mod1reference = output[3];
            mod3reference = output[5];
            mod2reference = output[4];
        }

        //set the zero of each module to be forward
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

        if (eff) {
            mod1reference=mod1efvalues[0];
            mod1power=mod1efvalues[1];
            mod2reference=mod2efvalues[0];
            mod2power=mod2efvalues[1];
            mod3reference=mod3efvalues[0];
            mod3power=mod3efvalues[1];
        }

        //change coax values into diffy values from pid and power
        double[] mod1values = mathsOperations.diffyConvert(mod1PID.pidOut(AngleUnit.normalizeDegrees(mod1reference-mod1P)),-mod1power);
        mod1m1.setPower(mod1values[0]);
        mod1m2.setPower(mod1values[1]);
        double[] mod2values = mathsOperations.diffyConvert(-mod2PID.pidOut(AngleUnit.normalizeDegrees(mod2reference-mod2P)),-mod2power);
        mod2m1.setPower(mod2values[0]);
        mod2m2.setPower(mod2values[1]);
        double[] mod3values = mathsOperations.diffyConvert(mod3PID.pidOut(AngleUnit.normalizeDegrees(mod3reference-mod3P)),mod3power);
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

        telemetry.addData("x",x);
        telemetry.addData("y",y);
        telemetry.addData("r",rot);
    }

    //tune module PIDs
    public void setPIDCoeffs(double Kp, double Kd,double Ki, double Kf){
        this.Kp = Kp;
        this.Kd = Kd;
        this.Ki = Ki;
        this.Kf = Kf;
    }

    //tunable module zeroing
    public void setModuleAdjustments(double module1Adjust, double module2Adjust, double module3Adjust){
        this.module1Adjust=module1Adjust;
        this.module2Adjust=module2Adjust;
        this.module3Adjust=module3Adjust;
    }
}
