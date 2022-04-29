package com.example.meepmeepmodule;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepSingle {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        System.setProperty("sun.java2d.opengl", "true");

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(12.3, 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(7.5, 63, Math.toRadians(90)))

                                .setReversed(true)
                                .splineTo(new Vector2d(-3.15, 41.42), Math.toRadians(-115))









                                .setReversed(false)
                                .lineToSplineHeading(new Pose2d(6, 57, Math.toRadians(0)))
                                .splineToLinearHeading(new Pose2d(20, 64, Math.toRadians(0)), Math.toRadians(0))
                                .forward(30)

                                .setReversed(true)
                                .back(30)
                                .splineTo(new Vector2d(-3.15, 41.42), Math.toRadians(-115))





                                .setReversed(false)
                                .lineToSplineHeading(new Pose2d(6, 57, Math.toRadians(0)))
                                .splineToLinearHeading(new Pose2d(20, 64, Math.toRadians(0)), Math.toRadians(0))
                                .forward(30)

                                .setReversed(true)
                                .back(30)
                                .splineTo(new Vector2d(-3.15, 41.42), Math.toRadians(-115))







                                .setReversed(false)
                                .lineToSplineHeading(new Pose2d(6, 57, Math.toRadians(0)))
                                .splineToLinearHeading(new Pose2d(20, 64, Math.toRadians(0)), Math.toRadians(0))
                                .forward(30)

                                .setReversed(true)
                                .back(30)
                                .splineTo(new Vector2d(-3.15, 41.42), Math.toRadians(-115))







                                .setReversed(false)
                                .lineToSplineHeading(new Pose2d(6, 57, Math.toRadians(0)))
                                .splineToLinearHeading(new Pose2d(20, 64, Math.toRadians(0)), Math.toRadians(0))
                                .forward(30)

                                .setReversed(true)
                                .back(30)
                                .splineTo(new Vector2d(-3.15, 41.42), Math.toRadians(-115))








                                .setReversed(false)
                                .lineToSplineHeading(new Pose2d(6, 57, Math.toRadians(0)))
                                .splineToLinearHeading(new Pose2d(20, 64, Math.toRadians(0)), Math.toRadians(0))
                                .forward(30)

                                .setReversed(true)
                                .back(30)
                                .splineTo(new Vector2d(-3.15, 41.42), Math.toRadians(-115))






                                .setReversed(false)
                                .lineToSplineHeading(new Pose2d(6, 57, Math.toRadians(0)))
                                .splineToLinearHeading(new Pose2d(20, 64, Math.toRadians(0)), Math.toRadians(0))
                                .forward(30)

                                .setReversed(true)
                                .back(30)
                                .splineTo(new Vector2d(-3.15, 41.42), Math.toRadians(-115))


                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}