package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Arrays;
import java.util.Vector;

public class MyClass {

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, java.lang.Math.toRadians(180), java.lang.Math.toRadians(180), 15.590464)
                .build();



        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-15, -63, Math.toRadians(00)))
                        .setTangent(Math.toRadians(120))
                        .splineToLinearHeading(new Pose2d(-58,-57, Math.toRadians(45)), Math.toRadians(225))
                .splineToConstantHeading(new Vector2d(-59,-58),Math.toRadians(225))
                        .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(-35,-24, Math.toRadians(90)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-42,-16,Math.toRadians(70)), Math.toRadians(160))
                        .setTangent(Math.toRadians(250))
                .lineToY(-56)
                        .setTangent(Math.toRadians(55))
                        .splineToLinearHeading(new Pose2d(-52,-15,Math.toRadians(80)), Math.toRadians(170))
                        .setTangent(Math.toRadians(260))
                .lineToY(-53)
                        .setTangent(Math.toRadians(75))
                        .splineToLinearHeading(new Pose2d(-28,-12, Math.toRadians(180)), Math.toRadians(0))
                        .setTangent(0)
                .lineToX(-24)
                .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}