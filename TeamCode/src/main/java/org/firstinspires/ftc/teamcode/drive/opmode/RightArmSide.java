//package org.firstinspires.ftc.teamcode.drive.opmode;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;




// Old Code - don't use




//@Autonomous(name = "DoubleWobblePark", group = "drive" )
//public class RightArmSide extends LinearOpMode {
//    @Override
//    public void runOpMode() {
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//
//        Pose2d startPose = new Pose2d(-63, -24, Math.toRadians(0));
//
//        drive.setPoseEstimate(startPose);
//
//        Trajectory driveToRing = drive.trajectoryBuilder(startPose)
//                .forward(36)
//                .build();
//
//        //scan rings here 1-3 = A to C, 0 = none sadge
//        int rings = (int) (Math.random() * 3) + 1;
//
//        //drive back to start if no rings detected
//        Trajectory backToStart = drive.trajectoryBuilder(driveToRing.end())
//                .back(-36)
//                .build();
//
//
//
//
//        Trajectory toZoneA = drive.trajectoryBuilder(driveToRing.end())
//                .forward(36)
//                .build();
//
//        //drop first wobble here
//
//        //turning is done outside of trajectories
//
//        Trajectory zoneAToWobble = drive.trajectoryBuilder(new Pose2d(9, -24, Math.toRadians(270)), true)
//                .splineTo(new Vector2d(-51, 12), Math.toRadians(180))
//                .build();
//
//        Trajectory wobbleToZoneA = drive.trajectoryBuilder(zoneAToWobble.end())
//                .splineTo(new Vector2d(33, -33), Math.toRadians(270))
//                .build();
//
//        //drop second wobble here
//
//        Trajectory zoneAToPark = drive.trajectoryBuilder(wobbleToZoneA.end(), true)
//                .splineTo(new Vector2d(9, -12), Math.toRadians(180))
//                .build();
//
//
//
//
//        Trajectory toZoneB = drive.trajectoryBuilder(driveToRing.end())
//                .splineTo(new Vector2d(36, 0), Math.toRadians(0.0))
//                .build();
//
//        Trajectory zoneBToWobble = drive.trajectoryBuilder(toZoneB.end(), true)
//                .splineTo(new Vector2d(-51, 12), Math.toRadians(180))
//                .build();
//
//        Trajectory wobbleToZoneB = drive.trajectoryBuilder(zoneBToWobble.end())
//                .splineTo(new Vector2d(30, 0), Math.toRadians(0))
//                .build();
//
//        Trajectory zoneBToPark = drive.trajectoryBuilder(wobbleToZoneB.end())
//                .back(21)
//                .build();
//
//
//
//
//        Trajectory toZoneC = drive.trajectoryBuilder(driveToRing.end())
//                .forward(87)
//                .build();
//
//        Trajectory zoneCToWobble = drive.trajectoryBuilder(toZoneC.end(), true)
//                .splineTo(new Vector2d(-51, 12), Math.toRadians(180))
//                .build();
//
//        Trajectory wobbleToZoneC = drive.trajectoryBuilder(zoneCToWobble.end())
//                .splineTo(new Vector2d(54, -24), Math.toRadians(0))
//                .build();
//
//        Trajectory zoneCToPark = drive.trajectoryBuilder(wobbleToZoneC.end())
//                .back(45)
//                .build();
//
//
//        waitForStart();
//
//        if(isStopRequested()) return;
//
//        drive.followTrajectory(driveToRing);
//
//        if(rings == 1) {
//            drive.followTrajectory(toZoneA);
//            drive.turn(Math.toRadians(-90));    //probably -90 b/c unit circle?
//            drive.followTrajectory(zoneAToWobble);
//            drive.followTrajectory(wobbleToZoneA);
//            drive.followTrajectory(zoneAToPark);
//        }
//        else if(rings == 2) {
//            drive.followTrajectory(toZoneB);
//            drive.followTrajectory(zoneBToWobble);
//            drive.followTrajectory(wobbleToZoneB);
//            drive.followTrajectory(zoneBToPark);
//        }
//        else if(rings == 3) {
//            drive.followTrajectory(toZoneC);
//            drive.followTrajectory(zoneCToWobble);
//            drive.followTrajectory(wobbleToZoneC);
//            drive.followTrajectory(zoneCToPark);
//        }
//        else {
//            drive.followTrajectory(backToStart);
//        }
//    }
//}
