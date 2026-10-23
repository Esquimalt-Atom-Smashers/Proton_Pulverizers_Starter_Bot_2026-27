package org.firstinspires.ftc.teamcode.Programs;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.sun.tools.javac.main.Option;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class Mecanum_Drive_TeleOp_Starter_Bot_OCT14 extends OpMode {

    private DcMotor frontRightMotor, frontLeftMotor, rearRightMotor, rearLeftMotor;

    private DcMotor intakeMotor;

    private DcMotorEx shooterMotor;

    private CRServo rightIntakeServo, leftIntakeServo;
    private CRServo transferServo;

    private IMU imu;



    //These are all the variables for the PIDF For the FlyWheel
    static final double TICKS_PER_REV = 28.0;
    static final double MAX_RPM = 6000.0;
    static final double MAX_TPS = MAX_RPM / 60 * TICKS_PER_REV; // 2800
    static final double NOMINAL_VOLTS = 12.0;

    // ---- Gains: edit these and redeploy ----
    public static double kF = 0.00038; // Feedforward gain
    public static double kP = 0.00060; // Proportional gain


    static final double RPM_STEP = 50;

    private VoltageSensor battery;
    private final ElapsedTime timer = new ElapsedTime();

    private double targetRPM = 1200;
    private double targetTPS;


    private boolean lastUp = false;
    private boolean lastDown = false;



    @Override
    public void init() {
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        rearRightMotor = hardwareMap.get(DcMotor.class, "rearRightMotor");
        rearLeftMotor = hardwareMap.get(DcMotor.class, "rearLeftMotor");

        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        rightIntakeServo = hardwareMap.get(CRServo.class, "frontRightServo");
        leftIntakeServo = hardwareMap.get(CRServo.class, "frontLeftServo");
        transferServo = hardwareMap.get(CRServo.class, "shooterServo");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rearLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        rightIntakeServo.setDirection(CRServo.Direction.FORWARD);
        leftIntakeServo.setDirection(CRServo.Direction.REVERSE);
        transferServo.setDirection(CRServo.Direction.FORWARD);

        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;


        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        battery = hardwareMap.voltageSensor.iterator().next();



        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // flywheel.setDirection(DcMotor.Direction.REVERSE);


        telemetry.addLine("D-pad Up/Down: target +/- " + (int) RPM_STEP + " RPM");
        telemetry.addLine("X: stop flywheel");
        telemetry.update();
    }

    @Override
    public void loop() {

        //Variables for the PIDF Control for the Flywheel
        boolean up = gamepad1.dpad_up;
        boolean down = gamepad1.dpad_down;

        if (up && !lastUp)     targetRPM = Math.min(MAX_RPM, targetRPM + RPM_STEP);
        if (down && !lastDown) targetRPM = Math.max(0.0  , targetRPM - RPM_STEP);

        targetTPS = targetRPM / 60 * TICKS_PER_REV;

        lastUp = up;
        lastDown = down;

        // Target velocity in encoder counts per second
        double targetTicksPerSec = (targetRPM / 60.0) * TICKS_PER_REV;

        // Current speed readings
        double currentTicksPerSec = shooterMotor.getVelocity();
        double currentRpm = (currentTicksPerSec / 60) * TICKS_PER_REV;

        // PF algorithm math
        double error = targetTicksPerSec - currentTicksPerSec;
        double feedForward = kF * targetTicksPerSec;
        double proportional = kP * error;

        // Prevent reversing by clamping lower bound to 0.0
        double commandedPower = Range.clip(feedForward + proportional, 0.0, 1.0);
        shooterMotor.setPower(commandedPower);



    //Field Centric Code
    double y  = -gamepad1.left_stick_y;
    double x  =  gamepad1.left_stick_x;
    double rx =  gamepad1.right_stick_x;

    double speed = gamepad1.right_bumper ? 0.4 : 1.0; // slow mode
        if (gamepad1.options || gamepad1.back) {
        imu.resetYaw();
    }

    double botHeading = getHeadingRadians();

    // Rotate the translation vector by -heading
    double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
    double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

    rotX *= 1.1; // counteract imperfect strafing

    // Normalize so no wheel exceeds 1.0 and ratios are preserved
    double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
    double flPower = (rotY + rotX + rx) / denominator;
    double blPower = (rotY - rotX + rx) / denominator;
    double frPower = (rotY - rotX - rx) / denominator;
    double brPower = (rotY + rotX - rx) / denominator;

    frontLeftMotor.setPower(flPower * speed);
    rearLeftMotor.setPower(blPower * speed);
    frontRightMotor.setPower(frPower * speed);
    rearRightMotor.setPower(brPower * speed);

    telemetry.addData("Heading (deg)", Math.toDegrees(botHeading));
    telemetry.addData("FL/FR", "%.2f / %.2f", flPower, frPower);
    telemetry.addData("BL/BR", "%.2f / %.2f", blPower, brPower);



        // ---- Telemetry ----
        double actualRPM = (shooterMotor.getVelocity() / 60.0) * TICKS_PER_REV;
        telemetry.addData("Target RPM", "%.0f", targetRPM);
        telemetry.addData("Actual RPM", "%.0f", actualRPM);
        telemetry.addData("Error RPM", "%.0f", targetRPM - actualRPM);
        telemetry.addData("Power", "%.3f", shooterMotor.getPower());
        telemetry.addData("Battery", "%.2f V", battery.getVoltage());
        telemetry.update();


//Below Here is the Old Open Loop Controls for the FlyWheel
        //telemetry.addData("Shooter Velocity Target", shooterTargetRPM);

    //this is for the servos on the side of the intake
    if(gamepad1.left_trigger>0.1){
        rightIntakeServo.setPower(1);
        leftIntakeServo.setPower(1);
    }else{
        rightIntakeServo.setPower(0);
        leftIntakeServo.setPower(0);
    }

    //this is for the transfer servo
    if(gamepad1.right_bumper){
        transferServo.setPower(1);

    }else if(gamepad1.left_bumper){
        transferServo.setPower(-1);
    } else {
        transferServo.setPower(0);
    }


    //Controls Intake Motor
    if(gamepad1.x){
        intakeMotor.setPower(-1);
    }else{
        intakeMotor.setPower(gamepad1.left_trigger);
    }


}

    @Override
    public void stop() {
    frontLeftMotor.setPower(0);
    rearLeftMotor.setPower(0);
    frontRightMotor.setPower(0);
    rearRightMotor.setPower(0);
    intakeMotor.setPower(0);
    shooterMotor.setPower(0);
    rightIntakeServo.setPower(0);
    leftIntakeServo.setPower(0);
    transferServo.setPower(0);
    }

    private double getHeadingRadians() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }
}

