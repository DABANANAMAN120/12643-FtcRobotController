package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


@TeleOp
public class DriveBaseTest extends LinearOpMode {

    private Motor frontLeft, frontRight, backLeft, backRight;
    private MecanumDrive drive;
    private GamepadEx driverOp;
/*
    @Override
    public void init() {

        drive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);
        driverOp = new GamepadEx(gamepad1);
    }
*/

    @Override
    public void runOpMode() {
        backLeft = new Motor(hardwareMap, "backLeft");
        backRight = new Motor(hardwareMap, "backRight");
        frontLeft = new Motor(hardwareMap, "frontLeft");
        frontRight = new Motor(hardwareMap, "frontRight");
        drive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);
        driverOp = new GamepadEx(gamepad1);
        TelemetryPacket packet;
        IMU imu;
        imu = hardwareMap.get(BHI260IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        sleep((long)2000);
        packet = new TelemetryPacket();
        packet.fieldOverlay()
                .setFill("blue")
                .fillRect(-20, -20, 40, 40);
        FtcDashboard dash = FtcDashboard.getInstance();
        while (!opModeIsActive()){
            packet.put("Yaw", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        }
        waitForStart();
        while(opModeIsActive()) {
            YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
            drive.driveFieldCentric(
                    driverOp.getLeftX(),
                    driverOp.getLeftY(),
                    gamepad1.right_stick_x,
                    angles.getYaw(AngleUnit.DEGREES)
            );
            packet.put("Yaw", angles.getYaw(AngleUnit.DEGREES));
            dash.sendTelemetryPacket(packet);

        }
    }

}