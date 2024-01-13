package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp
public class teaching extends LinearOpMode {
    DcMotor backRight, backLeft, frontRight, frontLeft;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        while(opModeIsActive()) {
            backLeft.setPower(-gamepad1.left_stick_y);
            backRight.setPower(-gamepad1.left_stick_y);
            frontRight.setPower(-gamepad1.left_stick_y);
            frontLeft.setPower(-gamepad1.left_stick_y);
        }
    }
}