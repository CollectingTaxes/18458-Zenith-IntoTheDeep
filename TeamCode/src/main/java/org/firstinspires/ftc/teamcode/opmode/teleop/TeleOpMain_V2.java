package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmode.command.arm.Reset;
import org.firstinspires.ftc.teamcode.opmode.command.arm.Score;
import org.firstinspires.ftc.teamcode.opmode.command.claw.Grab;
import org.firstinspires.ftc.teamcode.opmode.command.drive.DefaultDriveCommand;
import org.firstinspires.ftc.teamcode.opmode.command.drive.SlowDriveCommand;
import org.firstinspires.ftc.teamcode.opmode.command.slides.SlideHigh;
import org.firstinspires.ftc.teamcode.opmode.command.slides.SlideLow;
import org.firstinspires.ftc.teamcode.opmode.command.slides.SlideMid;
import org.firstinspires.ftc.teamcode.opmode.command.slides.SlideMoveManual;
import org.firstinspires.ftc.teamcode.opmode.command.slides.SlideReset;
import org.firstinspires.ftc.teamcode.subsystem.CommandBased.Arm;
import org.firstinspires.ftc.teamcode.subsystem.CommandBased.Claw;
import org.firstinspires.ftc.teamcode.subsystem.CommandBased.SlideV2;
import org.firstinspires.ftc.teamcode.subsystem.DriveSub.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystem.DriveSub.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.GamepadTrigger;
import org.firstinspires.ftc.teamcode.util.MatchOpMode;

@Config
@TeleOp
public class TeleOpMain_V2 extends MatchOpMode {

    private GamepadEx driverGamepad; //Driver 1
    private GamepadEx operatorGamepad; // Driver 2

    private Claw claw;
    private SlideV2 slide;
    private Arm arm;
    private Drivetrain drivetrain;

    //Drive drive = new Drive(this);

    @Override
    public void robotInit() {

        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        arm = new Arm(hardwareMap, telemetry);
        slide = new SlideV2(hardwareMap, telemetry);
        claw = new Claw(hardwareMap, telemetry);

        drivetrain = new Drivetrain(new MecanumDrive(hardwareMap, telemetry, true), telemetry, hardwareMap);
        drivetrain.init();

    }

    @Override
    public void configureButtons() {

        drivetrain.setDefaultCommand(new DefaultDriveCommand(drivetrain, driverGamepad, false));

        //Button recenterIMU = (new GamepadButton(driverGamepad, GamepadKeys.Button.A))
                //.whenPressed(new InstantCommand(drivetrain::reInitializeIMU));

        Button recenterIMU2 = (new GamepadButton(driverGamepad, GamepadKeys.Button.B))
                .whenPressed(new InstantCommand(drivetrain::reInitializeIMU));

        Button slowMode = (new GamepadButton(driverGamepad, GamepadKeys.Button.LEFT_BUMPER))
                .whileHeld(new SlowDriveCommand(drivetrain, driverGamepad, false));

        slide.setDefaultCommand(new SlideMoveManual(slide, operatorGamepad::getLeftY));

        Button slideReset = new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new SlideReset(slide, claw, arm));

        Button slideLow = new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new SlideLow(slide, claw, arm));

        Button slideMid = new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new SlideMid(slide, claw, arm));

        Button slideHigh = new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_UP)
                .whenPressed(new SlideHigh(slide, claw, arm));

        Button Score = new GamepadButton(operatorGamepad, GamepadKeys.Button.A)
                .whenPressed(new Score(arm));

        Button Reset = new GamepadButton(operatorGamepad, GamepadKeys.Button.B)
                .whenPressed(new Reset(arm));

        Button Claw = new GamepadButton(operatorGamepad, GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new Grab(claw));
        Button ClawOuttake = new GamepadButton(operatorGamepad, GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new org.firstinspires.ftc.teamcode.opmode.command.claw.Reset(claw));

    }
    @Override
    public void matchStart() {

    }

}
