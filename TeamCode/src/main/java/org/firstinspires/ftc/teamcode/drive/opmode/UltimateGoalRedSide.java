package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "DoubleWobblePark", group = "drive" )
public class UltimateGoalRedSide extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-63, -24, Math.toRadians(0));

        drive.setPoseEstimate(startPose);



        //scan rings here 0, 1, 4 = A, B, C
        int rings = 0;





        Trajectory toZoneA = drive.trajectoryBuilder(startPose)
                .forward(46)
                .splineTo(new Vector2d(7, -34), Math.toRadians(305))
                .build();


        Trajectory zoneAToDown = drive.trajectoryBuilder(toZoneA.end(), true)
                .splineTo(new Vector2d(-15, -24), Math.toRadians(180))
                .back(37)
                .build();


        Trajectory downToZoneA = drive.trajectoryBuilder(zoneAToDown.end())
                .forward(26)
                .splineTo(new Vector2d(-2, -34), Math.toRadians(305))
                .build();



        Trajectory zoneAToPark = drive.trajectoryBuilder(downToZoneA.end(), true)
                .splineTo(new Vector2d(9, -12), Math.toRadians(0))
                .build();




        Trajectory toZoneB = drive.trajectoryBuilder(startPose)
                .forward(102)
                .build();


        Trajectory zoneBToDown = drive.trajectoryBuilder(toZoneB.end())
                .back(91)
                .build();


        Trajectory downToZoneB = drive.trajectoryBuilder(zoneBToDown.end())
                .forward(82)
                .build();


        Trajectory zoneBToPark = drive.trajectoryBuilder(downToZoneB.end())
                .back(21)
                .build();




        Trajectory toZoneC = drive.trajectoryBuilder(startPose)
                .forward(94)
                .splineTo(new Vector2d(55, -34), Math.toRadians(305))
                .build();

        Trajectory zoneCToDown = drive.trajectoryBuilder(toZoneC.end(), true)
                .splineTo(new Vector2d(31, -24), Math.toRadians(180))
                .back(83)
                .build();

        Trajectory downToZoneC = drive.trajectoryBuilder(zoneCToDown.end())
                .forward(74)
                .splineTo(new Vector2d(46, -34), Math.toRadians(305))
                .build();

        Trajectory zoneCToPark = drive.trajectoryBuilder(downToZoneC.end(), true)
                .splineTo(new Vector2d(22, -24), Math.toRadians(180))
                .back(13)
                .build();




        Trajectory downToWobble = drive.trajectoryBuilder(new Pose2d(-52, -24, Math.toRadians(0)), false)
                .strafeLeft(12.0)
                .build();

        Trajectory wobbleToDown = drive.trajectoryBuilder(new Pose2d(-52, -12, Math.toRadians(0)), false)
                .strafeRight(12.0)
                .build();




        waitForStart();

        if(isStopRequested()) return;

        if(rings == 0) {
            drive.followTrajectory(toZoneA);
            drive.followTrajectory(zoneAToDown);
            drive.followTrajectory(downToWobble);
            drive.followTrajectory(wobbleToDown);
            drive.followTrajectory(downToZoneA);
            drive.followTrajectory(zoneAToPark);
        }
        else if(rings == 1) {
            drive.followTrajectory(toZoneB);
            drive.followTrajectory(zoneBToDown);
            drive.followTrajectory(downToWobble);
            drive.followTrajectory(wobbleToDown);
            drive.followTrajectory(downToZoneB);
            drive.followTrajectory(zoneBToPark);
        }
        else if(rings == 4) {
            drive.followTrajectory(toZoneC);
            drive.followTrajectory(zoneCToDown);
            drive.followTrajectory(downToWobble);
            drive.followTrajectory(wobbleToDown);
            drive.followTrajectory(downToZoneC);
            drive.followTrajectory(zoneCToPark);
        }
    }
}
