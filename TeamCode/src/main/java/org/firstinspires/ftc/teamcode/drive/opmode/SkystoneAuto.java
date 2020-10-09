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

        Trajectory collection = drive.trajectoryBuilder(startPose)
                .forward(24)
                .splineTo(new Vector2d(-43, 24), Math.toRadians(235))
                .build();

        Trajectory deposit = drive.trajectoryBuilder(collection.end(), true)
                .splineTo(new Vector2d(48, 32), Math.toRadians(270))
                .build();

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(collection);
        drive.followTrajectory(deposit);
    }
}
