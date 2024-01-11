package org.firstinspires.ftc.teamcode.opmode.auto.blueAutos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.Arm;
import org.firstinspires.ftc.teamcode.subsystem.Drive;
import org.firstinspires.ftc.teamcode.subsystem.Slides;
import org.firstinspires.ftc.teamcode.subsystem.Vision;
import org.firstinspires.ftc.teamcode.subsystem.pipelines.ColorDetection;
import org.firstinspires.ftc.teamcode.subsystem.pipelines.ColorDetectionBlue;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.TrajectorySequence;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;

@Autonomous (name = "RightAutoBlue", group = "RoadRunnerPath")
public class LeftAutoBlue extends LinearOpMode {
    ColorDetectionBlue colorDetectionBlue = new ColorDetectionBlue();
    public static double Park_Distance = 80;
    public static double Spike_Dis = 29;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        Drive drive = new Drive(this);
        Arm arm = new Arm(this);
        Slides slides = new Slides(this);
        ColorDetectionBlue colorDetectionBlue = new ColorDetectionBlue();
        Vision vision = new Vision(this, Vision.AllianceColor.BLUE);
        Pose2d startPos = new Pose2d(-34, 68, Math.toRadians(90));
        ElapsedTime timer = new ElapsedTime();

        drive.setPoseEstimate(startPos);

        TrajectorySequence Trajectory1 = drive.trajectorySequenceBuilder(startPos)
                .forward(Spike_Dis)
                .build();

        TrajectorySequence Trajectory2 = drive.trajectorySequenceBuilder(Trajectory1.end())
                .turn(Math.toRadians(90))
                .forward(Park_Distance)
                .build();
        //Run Auto

        switch (colorDetectionBlue.getLocation()) {
            case LEFT:
                drive.followTrajectorySequence(Trajectory1);
                break;
            case CENTER:
                drive.followTrajectorySequence(Trajectory1);
                drive.followTrajectorySequence(Trajectory2);
                break;
            case RIGHT:
                drive.followTrajectorySequence(Trajectory2);
                break;

        }
    }
}
