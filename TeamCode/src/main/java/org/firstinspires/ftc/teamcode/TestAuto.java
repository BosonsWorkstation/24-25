package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class TestAuto extends LinearOpMode {
    private static final SampleMecanumDrive.DirectionEnum direction = SampleMecanumDrive.DirectionEnum.WEST;
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap, telemetry,direction);

        Pose2d startPose = new Pose2d(0,0,0);

        drive.setPoseEstimate(startPose);

        TrajectorySequence Traj1 = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(10,0),0)
                .splineTo(new Vector2d(11,5),0)
                .build();




        waitForStart();


        drive.followTrajectorySequence(Traj1);






    }

}
