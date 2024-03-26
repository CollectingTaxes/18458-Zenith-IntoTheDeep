package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class BlueClose {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(650);
        Double leftScore = 28.0;
        Double rightScore = 44.0;
        Double centerScore = 36.0;

        Pose2d closeLeftSpike = new Pose2d(18, 39, Math.toRadians(90));
        Pose2d closeCenterSpike = new Pose2d(12, 32, Math.toRadians(90));
        Pose2d closeRightSpike = new Pose2d(6, 39, Math.toRadians(75));

        Pose2d closeLeftScore = new Pose2d(49, 44, Math.toRadians(180));
        Pose2d closeCenterScore = new Pose2d(49, 36, Math.toRadians(180));
        Pose2d closeRightScore = new Pose2d(49, 28, Math.toRadians(180));

        Vector2d ParkLeft = new Vector2d(45, 60);
        Vector2d ParkRight = new Vector2d(45, 12);

        RoadRunnerBotEntity blueAllianceLeft = new DefaultBotBuilder(meepMeep)
                .setColorScheme((new ColorSchemeBlueLight()))
                .setDimensions(14, 16)
                .setConstraints(52.48291908330528, 52.48291908330528, Math.toRadians(180), Math.toRadians(270), 11.15)
                .followTrajectorySequence(driveShim ->
                                driveShim.trajectorySequenceBuilder(new Pose2d(17, 63, Math.toRadians(90)))
                                        .lineToSplineHeading(closeRightSpike)
                                        //.strafeRight(5)
                                        .lineToSplineHeading(closeRightScore)

                                        .waitSeconds(1)

                                        //Cycle 1 Start
                                        .strafeRight(5)
                                        .splineToConstantHeading(new Vector2d(7, 60), Math.toRadians(180))
                                        .forward(25)
                                        .splineToConstantHeading(new Vector2d(-56, 35), Math.toRadians(180))

                                        //Intake
                                        .waitSeconds(1)

                                        //Cycle 1 End
                                        .back(5)
                                        .splineToConstantHeading(new Vector2d(-24, 60), Math.toRadians(0))
                                        .back(35)
                                        .splineToConstantHeading(new Vector2d(49, 36), Math.toRadians(0))

                                        //Score
                                        .waitSeconds(1)

                                        .waitSeconds(1)

                                        .lineToConstantHeading(ParkRight)
                                        .build()
                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(blueAllianceLeft)
                .start();
    }
}
