package org.firstinspires.ftc.teamcode.Programs;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.sun.tools.javac.main.Option;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Mecanum_Drive_TeleOp_Starter_Bot_OCT14 extends OpMode {

    private DcMotor frontRightMotor, frontLeftMotor, rearRightMotor, rearLeftMotor;

    private DcMotor intakeMotor;

    private DcMotorEx shooterMotor;

    private Servo rightIntakeServo, leftIntakeServo;
    private Servo transferServo;

    private IMU imu;

    private double shooterTargetRPM;
    private double shooterTargetTPS;

    private double shooterActualRPM;

    @Override
    public void init() {
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        rearRightMotor = hardwareMap.get(DcMotor.class, "rearRightMotor");
        rearLeftMotor = hardwareMap.get(DcMotor.class, "rearRightMotor");

        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        rightIntakeServo = hardwareMap.get(Servo.class, "rightIntakeServo");
        leftIntakeServo = hardwareMap.get(Servo.class, "leftIntakeServo");
        transferServo = hardwareMap.get(Servo.class, "transferServo");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rearLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        rightIntakeServo.setDirection(Servo.Direction.FORWARD);
        leftIntakeServo.setDirection(Servo.Direction.FORWARD);
        transferServo.setDirection(Servo.Direction.FORWARD);

        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;


        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        shooterTargetRPM = 3000;
    }

    @Override
    public void loop() {

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
    telemetry.addData("Shooter Velocity Target", shooterTargetRPM);

    //this is for the servos on the side of the intake
    if(gamepad1.a){
        rightIntakeServo.setPosition(1);
        leftIntakeServo.setPosition(1);
    }else{
        rightIntakeServo.setPosition(0);
        leftIntakeServo.setPosition(0);
    }

    //Controls Intake Motor
    if(gamepad1.x){
        intakeMotor.setPower(-1);
    }else{
        intakeMotor.setPower(gamepad1.left_trigger);
    }

    shooterTargetTPS = (shooterTargetRPM/60)*28;

    //Start and Stop of Shooter Motor
    if(gamepad1.right_bumper){
        shooterMotor.setVelocity(shooterTargetTPS);
    }else {
        shooterMotor.setVelocity(0);
    }

    //Controls for setting Shooter Target RPM
    if(gamepad1.dpad_up){
        shooterTargetRPM = shooterTargetRPM + 200;
    }

    if(gamepad1.dpad_down){
        shooterTargetRPM = shooterTargetRPM - 200;
    }

    shooterActualRPM = (shooterMotor.getVelocity()/28)*60;

    telemetry.addData("Shooter Actual RPM", shooterActualRPM);
}

    @Override
    public void stop() {
    frontLeftMotor.setPower(0);
    rearLeftMotor.setPower(0);
    frontRightMotor.setPower(0);
    rearRightMotor.setPower(0);
    intakeMotor.setPower(0);
    shooterMotor.setPower(0);
    rightIntakeServo.setPosition(0);
    leftIntakeServo.setPosition(0);
    transferServo.setPosition(0);
    }

    private double getHeadingRadians() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }
}

