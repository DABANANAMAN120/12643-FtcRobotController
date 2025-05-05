package org.firstinspires.ftc.teamcode;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.ExportToBlocks;

public class FTCLib_Drivetrain extends BlocksOpModeCompanion {
    public static MecanumDrive mecanum;

    @ExportToBlocks(

    )
    public static void InitMecanum (Motor bLeft, Motor bRight, Motor fLeft, Motor fRight) {
        MecanumDrive mecanum = new MecanumDrive(fLeft, fRight, bLeft, bRight);
    }
    @ExportToBlocks(

    )
    public static void RunMecanum () {
        mecanum.driveFieldCentric(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, IMU_Settings.IMU_Read());
    }
}