package org.firstinspires.ftc.teamcode.Programs;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class Noahs_TeleOp extends OpMode {


    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backRightDrive;
    DcMotor backLeftDrive;

    DcMotor intakeMotor;

    DcMotor shooterMotor;


    IMU imu;


    @Override
    public void init() {
        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backRightDrive = hardwareMap.get(DcMotor.class, "rearRightMotor");
        backLeftDrive = hardwareMap.get(DcMotor.class, "rearLeftMotor");

        backLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;


        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

    }

    @Override
    public void loop() {
        double shooterWheelSpeed = shooterMotor.getPower();
        telemetry.addLine("Press A to reset Yaw");
        telemetry.addLine("Hold left bumper to drive in robot relative");
        telemetry.addLine("The left joystick sets the robot direction");
        telemetry.addLine("Moving the right joystick left and right turns the robot");
        telemetry.addLine("Shooter Speed:" );


        if (gamepad1.a) {
            imu.resetYaw();
        }

        if (gamepad1.left_bumper) {
            drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        } else {
            driveFieldRelative(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
        }
        intakeMotor.setPower(gamepad1.left_trigger);
    }

    private void driveFieldRelative(double  forward, double right, double rotate) {

        double theta = Math.atan2(forward, right);
        double r = Math.hypot(forward, right);

        theta = AngleUnit.normalizeRadians(theta -
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        drive(newForward, newRight, rotate);


    }
        public void drive(double forward, double right, double rotate) {


            double frontLeftPower = forward + right + rotate;
            double frontRightPower = forward - right - rotate;
            double backRightPower = forward + right - rotate;
            double backLeftPower = forward - right + rotate;

            double maxpower = 1.0;
            double maxspeed = 1.0;
            maxpower = Math.max(maxpower, Math.abs(frontLeftPower));
            maxpower = Math.max(maxpower, Math.abs(frontRightPower));
            maxpower = Math.max(maxpower, Math.abs(backRightPower));
            maxpower = Math.max(maxpower, Math.abs(backLeftPower));

            frontLeftDrive.setPower (maxspeed * (frontLeftPower / maxpower));
            frontRightDrive.setPower (maxspeed * (frontRightPower / maxpower));
            backLeftDrive.setPower (maxspeed * (backLeftPower / maxpower));
            backRightDrive.setPower (maxspeed * (backRightPower / maxpower));










    }
}