package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Robot;

public class NewAutoPathing {
    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(650);
        RoadRunnerBotEntity Robot = new DefaultBotBuilder(meepMeep)
                .setColorScheme((new ColorSchemeBlueLight()))
                .setDimensions(13, 14)
                .setConstraints(30, 40, Math.toRadians(180), Math.toRadians(270), 11.15)
                .followTrajectorySequence(driveShim ->
                        driveShim.trajectorySequenceBuilder(new Pose2d(7, -63, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(15, -39, Math.toRadians(45)))
                                .lineToLinearHeading(new Pose2d(49, -44, Math.toRadians(175)))
                                .build()

                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(Robot)
                .start();
    }
}
