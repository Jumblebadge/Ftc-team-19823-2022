package com.example.meepmeepvisual;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepVisual {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(10, 0, 0))
                                .forward(20)
                                .turn(Math.toRadians(45))
                                .forward(30)
                                .turn(Math.toRadians(135))
                                .forward(30)
                                .turn(Math.toRadians(90))
                                .forward(30)
                                .turn(Math.toRadians(90))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}