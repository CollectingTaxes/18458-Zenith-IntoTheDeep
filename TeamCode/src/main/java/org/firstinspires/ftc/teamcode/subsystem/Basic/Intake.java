package org.firstinspires.ftc.teamcode.subsystem.Basic;

import android.widget.Spinner;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Intake {
    public DcMotor spinner;
    public Servo leftServoIntake, rightServoIntake;
    public final Gamepad gamepad2;
    public final Gamepad gamepad1;
    public HardwareMap hardwareMap;
    public static volatile double OUTTAKE = -0.5;
    public static volatile double INTAKE = 0.65;
    public static volatile double DROPED_DOWN = 0;
    public static volatile double DROPPED_UP = 1;
    public double NOTHING = 0;
    public Intake(OpMode opMode) {
        this.hardwareMap = opMode.hardwareMap;
        this.gamepad2 = opMode.gamepad2;
        this.gamepad1 = opMode.gamepad1;

        spinner = (DcMotor) hardwareMap.get("Intake");
        spinner.setDirection(DcMotorSimple.Direction.REVERSE);

        leftServoIntake = (Servo) hardwareMap.get("leftServoIntake");
        rightServoIntake = (Servo) hardwareMap.get("rightServoIntake");
        leftServoIntake.setDirection(Servo.Direction.REVERSE);
        rightServoIntake.setDirection(Servo.Direction.FORWARD);
    }
    public void teleOp() {
        if (gamepad2.right_trigger >= 0.5) spinnerIntakeThing(INTAKE);
        else if (gamepad2.left_trigger >= 0.5) spinnerIntakeThing(OUTTAKE);
        else if (gamepad2.a) dropDownIntake(DROPED_DOWN);
        else if (gamepad2.b) dropDownIntake(DROPPED_UP);
        else spinnerIntakeThing(NOTHING);
    }
    public void soloTeleOp() {
        if (gamepad1.right_bumper && gamepad1.right_trigger <=1) spinnerIntakeThing(INTAKE);
        else if (gamepad1.left_bumper && gamepad1.left_trigger <=1) spinnerIntakeThing(OUTTAKE);
        else spinnerIntakeThing(NOTHING);
    }
    public void spinnerIntakeThing(double powerSpinner) {
        spinner.setPower(powerSpinner);
    }
    public void dropDownIntake(double position) {
        leftServoIntake.setPosition(position);
        rightServoIntake.setPosition(position);
    }
    public void spinnerAutoThing(double powerSpinner, long durationOfActions) throws InterruptedException {
        spinner.setPower(powerSpinner);
        Thread.sleep(durationOfActions);
    }
}