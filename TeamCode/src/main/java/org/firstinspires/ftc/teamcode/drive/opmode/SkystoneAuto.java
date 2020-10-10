package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "SkystoneAuto", group = "drive" )
public class SkystoneAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-33, 63, Math.toRadians(270));

        drive.setPoseEstimate(startPose);

        Trajectory collection1 = drive.trajectoryBuilder(startPose)
                .forward(24)
                .splineTo(new Vector2d(-43, 24), Math.toRadians(235))
                .build();

        Trajectory deposit1 = drive.trajectoryBuilder(collection1.end(), true)
                .splineTo(new Vector2d(48, 32), Math.toRadians(270))
                .build();

        Trajectory collection2 = drive.trajectoryBuilder(deposit1.end())
                .splineTo(new Vector2d(-34.0, 24.0), Math.toRadians(235))
                .build();

        Trajectory deposit2 = drive.trajectoryBuilder(collection2.end(), true)
                .splineTo(new Vector2d(48, 32), Math.toRadians(270))
                .build();

        Trajectory collection3 = drive.trajectoryBuilder(deposit2.end())
                .splineTo(new Vector2d(-25.0, 24.0), Math.toRadians(235))
                .build();

        Trajectory deposit3 = drive.trajectoryBuilder(collection3.end(), true)
                .splineTo(new Vector2d(48, 32), Math.toRadians(270))
                .build();

        Trajectory park = drive.trajectoryBuilder(deposit3.end())
                .strafeLeft(48)
                .build();

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(collection1);
        drive.followTrajectory(deposit1);
        drive.followTrajectory(collection2);
        drive.followTrajectory(deposit2);
        drive.followTrajectory(collection3);
        drive.followTrajectory(deposit3);
        drive.followTrajectory(park);
    }
}
