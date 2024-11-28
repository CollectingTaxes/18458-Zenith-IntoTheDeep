package org.firstinspires.ftc.teamcode.subsystem.CommandBased;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Arm extends SubsystemBase {

    public static boolean REVERSED = false;

<<<<<<< HEAD
    public static double resetPose = 0.8,
                                grabPose = 0.43;
=======
    public static double resetPose = 0.5,
                                grabPose = 0.25;
>>>>>>> parent of c62d4ea (Merge remote-tracking branch 'origin/main')
    Telemetry telemetry;
    private final SimpleServo leftArm, rightArm;

    public Arm(final HardwareMap hMap, Telemetry telemetry) {
        this.leftArm = new SimpleServo(hMap, "leftArm", 0, 360);
        this.rightArm = new SimpleServo(hMap, "rightArm", 0, 360);

        leftArm.setInverted(REVERSED);
        rightArm.setInverted(true);

        leftArm.setPosition(resetPose);
        rightArm.setPosition(resetPose);

        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {
        telemetry.addData("ArmPose", leftArm.getPosition());
    }

    public void grab() {
        leftArm.setPosition(grabPose);
        rightArm.setPosition(grabPose);
    }

    public void reset() {
        leftArm.setPosition(resetPose);
        rightArm.setPosition(resetPose);
    }
}
