package org.firstinspires.ftc.teamcode.opmodes;

//Import EVERYTHING we need
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;import com.outoftheboxrobotics.photoncore.PhotonCore;import com.acmerobotics.dashboard.config.Config;import com.qualcomm.robotcore.hardware.AnalogInput;import com.acmerobotics.dashboard.FtcDashboard;import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;import com.qualcomm.hardware.bosch.BNO055IMU;import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;import com.qualcomm.robotcore.hardware.DcMotorEx;import com.qualcomm.robotcore.hardware.CRServo;import com.qualcomm.robotcore.util.ElapsedTime;import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;import org.firstinspires.ftc.robotcore.external.navigation.Orientation;import org.firstinspires.ftc.teamcode.maths.controlLoopMath;import org.firstinspires.ftc.teamcode.maths.mathsOperations;import org.firstinspires.ftc.robotcore.external.navigation.Position;import org.firstinspires.ftc.teamcode.maths.swerveMaths;import java.util.List;import org.firstinspires.ftc.robotcore.external.navigation.Velocity;import com.qualcomm.hardware.lynx.LynxModule;

@Config
@TeleOp(name="godSwerve", group="Linear Opmode")
public class godSwerve extends LinearOpMode {

    //Initialize all of our hardware
    private AnalogInput mod1E = null, mod2E = null, mod3E = null;

    private DcMotorEx mod1m2 = null, mod2m2 = null, mod3m2 = null;

    private DcMotorEx mod1m1 = null, mod2m1 = null, mod3m1 = null;

    List<LynxModule> allHubs = null;

    //Initialize FTCDashboard
    FtcDashboard dashboard;

    //Define reference variables for modules' heading
    public static double mod1reference=0,mod2reference=0,mod3reference=0;
    public static double mod1reference1=0,mod2reference1=0,mod3reference1=0;

    //Timers for the PID loops
    ElapsedTime mod3timer =  new ElapsedTime(); ElapsedTime mod2timer =  new ElapsedTime(); ElapsedTime mod1timer =  new ElapsedTime();

    //Define module position variables
    double mod1P = 0, mod2P = 0, mod3P = 0;

    //Define variables for power of wheels
    double mod1power = 0,mod2power = 0,mod3power = 0;

    //Tuning values so that wheels are always facing straight (accounts for encoder drift - tuned manually)
    public static double mod3PC = -5, mod1PC = 87, mod2PC = 190;

    //IMU
    BNO055IMU IMU;
    Orientation angles;

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        //Calibrate the IMU
        //CHANGE TO ODO HEADING!
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        IMU = hardwareMap.get(BNO055IMU.class, "IMU");
        IMU.initialize(parameters);
        IMU.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        //Initialize FTCDashboard telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //Link all of our hardware to our hardwaremap
        //E = encoder, m1 = motor 1, m2 = motor 2
        mod1E = hardwareMap.get(AnalogInput.class, "mod1E");
        mod2E = hardwareMap.get(AnalogInput.class, "mod2E");
        mod3E = hardwareMap.get(AnalogInput.class, "mod3E");

        mod1m1 = hardwareMap.get(DcMotorEx.class, "mod1m1");
        mod2m1 = hardwareMap.get(DcMotorEx.class, "mod2m1");
        mod3m1 = hardwareMap.get(DcMotorEx.class, "mod3m1");

        mod1m2 = hardwareMap.get(DcMotorEx.class, "mod1m2");
        mod2m2 = hardwareMap.get(DcMotorEx.class, "mod2m2");
        mod3m2 = hardwareMap.get(DcMotorEx.class, "mod3m2");

        //Bulk sensor reads
        allHubs = hardwareMap.getAll(LynxModule.class);

        //Initialize FTCDashboard
        dashboard = FtcDashboard.getInstance();

        //Create objects for the classes we use for swerve
        swerveMaths swavemath = new swerveMaths();

        controlLoopMath mod1PID = new controlLoopMath(0.2,0.0001,0,0,mod1timer);
        controlLoopMath mod2PID = new controlLoopMath(0.2,0.0001,0,0,mod2timer);
        controlLoopMath mod3PID = new controlLoopMath(0.2,0.0001,0,0,mod3timer);

        //Bulk sensor reads
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        PhotonCore.enable();

        //the wraparound detection variables
        boolean mod1wrapped = false;
        double mod1lastpos = 0;
        boolean mod2wrapped = false;
        double mod2lastpos = 0;
        boolean mod3wrapped = false;
        double mod3lastpos = 0;

        waitForStart();
        while (opModeIsActive()) {

            //Clear the cache for better loop times (bulk sensor reads)
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            //Turn our MA3 absolute encoder signals from volts to degrees
            //detecting wraparounds on the ma3's so that the 1:2 gear ratio does not matter
            double mod1P1 = mod1E.getVoltage() * -74.16;
            double mod2P1 = mod2E.getVoltage() * -74.16;
            double mod3P1 = mod3E.getVoltage() * -74.16;

            double mod1positiondelta = mod1P1 - mod1lastpos;
            mod1lastpos = mod1P1;

            mod1wrapped = ((mod1positiondelta > 180) != mod1wrapped);
            mod1wrapped = ((mod1positiondelta <-180) != mod1wrapped);
            mod1P = (mod1wrapped == true ? 180 + mod1P1/2 : mod1P1/2);

            double mod2positiondelta = mod2P1 - mod2lastpos;
            mod2lastpos = mod2P1;

            mod2wrapped = ((mod2positiondelta > 180) != mod2wrapped);
            mod2wrapped = ((mod2positiondelta <-180) != mod2wrapped);
            mod2P = (mod2wrapped == true ? 180 + mod2P1/2 : mod2P1/2);

            double mod3positiondelta = mod3P1 - mod3lastpos;
            mod3lastpos = mod3P1;

            mod3wrapped = ((mod3positiondelta > 180) != mod3wrapped);
            mod3wrapped = ((mod3positiondelta <-180) != mod3wrapped);
            mod3P = (mod3wrapped == true ? 180 + mod3P1/2 : mod3P1/2);

            //Update heading of robot
            angles   = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double heading = angles.firstAngle*-1;

            telemetry.addData("IMU",heading);

            //Anglewrap our positions and references for each wheel
            mod1P = mathsOperations.angleWrap(mod1P);
            mod2P = mathsOperations.angleWrap(mod2P);
            mod3P = mathsOperations.angleWrap(mod3P);

            mod1reference = mathsOperations.angleWrap(mod1reference);
            mod2reference = mathsOperations.angleWrap(mod2reference);
            mod3reference = mathsOperations.angleWrap(mod3reference);

            //Retrieve the angles and powers for all of our wheels from the swerve kinematics
            double[] output = swavemath.Math(gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x,heading,true);
            mod1power=output[0];
            mod3power=output[2];
            mod2power=output[1];

            if (gamepad1.left_stick_y!=0||gamepad1.left_stick_x!=0||gamepad1.right_stick_x!=0){

                mod1reference1=output[3];
                mod3reference1=output[5];
                mod2reference1=output[4];

            }
            mod1reference=mod1reference1;
            mod3reference=mod3reference1;
            mod2reference=mod2reference1;

            mod1reference = mathsOperations.angleWrap(mod1reference);
            mod2reference = mathsOperations.angleWrap(mod2reference);
            mod3reference = mathsOperations.angleWrap(mod3reference);

            //Subtract our tuning values to account for any encoder drift
            mod3reference -= mod3PC;
            mod2reference -= mod2PC;
            mod1reference -= mod1PC;

            mod1reference = mathsOperations.angleWrap(mod1reference);
            mod2reference = mathsOperations.angleWrap(mod2reference);
            mod3reference = mathsOperations.angleWrap(mod3reference);

            //Run our powers, references, and positions through efficient turning code for each wheel and get the new values
            double[] mod1efvalues = mathsOperations.efficientTurn(mod1reference,mod1P,mod1power);
            mod1reference=mod1efvalues[0];
            mod1power=mod1efvalues[1];

            double[] mod2efvalues = mathsOperations.efficientTurn(mod2reference,mod2P,mod2power);
            mod2reference=mod2efvalues[0];
            mod2power=mod2efvalues[1];

            double[] mod3efvalues = mathsOperations.efficientTurn(mod3reference,mod3P,mod3power);
            mod3reference=mod3efvalues[0];
            mod3power=mod3efvalues[1];

            double[] mod1values = mathsOperations.diffyConvert(mod1PID.PIDout(mod1reference,mod1P),mod1power);
            double mod1m1power = mod1values[0];
            double mod1m2power = mod1values[1];
            double[] mod2values = mathsOperations.diffyConvert(mod2PID.PIDout(mod2reference,mod2P),mod2power);
            double mod2m1power = mod2values[0];
            double mod2m2power = mod2values[1];
            double[] mod3values = mathsOperations.diffyConvert(mod3PID.PIDout(mod3reference,mod3P),mod3power);
            double mod3m1power = mod3values[0];
            double mod3m2power = mod3values[1];

            //Use our controlLoopMath class to find the power needed to go into our CRservo to achieve our desired target
            mod1m1.setPower(mod1m1power);
            mod1m2.setPower(mod1m2power);

            mod2m1.setPower(mod2m1power);
            mod2m2.setPower(mod2m2power);

            mod3m1.setPower(mod3m1power);
            mod3m2.setPower(mod3m2power);

            telemetry.addData("mod1reference",mod1reference);
            telemetry.addData("mod2reference",mod2reference);
            telemetry.addData("mod3reference",mod3reference);

            telemetry.addData("mod1P",mod1P);
            telemetry.addData("mod2P",mod2P);
            telemetry.addData("mod3P",mod3P);

            telemetry.addData("mod3power",mod3power);
            telemetry.addData("mod2power",mod2power);
            telemetry.addData("mod1power",mod1power);
            telemetry.update();

        }
    }
}