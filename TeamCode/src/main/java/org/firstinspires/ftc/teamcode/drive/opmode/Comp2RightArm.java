package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.Arm;
import org.firstinspires.ftc.teamcode.drive.Camera;
import org.firstinspires.ftc.teamcode.drive.Intake;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.Shooter;

@Autonomous(name = "RightArmShoot", group = "drive" )
public class Comp2RightArm extends LinearOpMode {

    public Intake intake;
    public Shooter shooter;
    public Arm arm;
    public Camera camera;

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        intake = new Intake(hardwareMap.dcMotor.get("intakeMotor"), hardwareMap.servo.get("legsOfDoom"));
        shooter = new Shooter(hardwareMap.dcMotor.get("shooter"));
        arm = new Arm(hardwareMap.dcMotor.get("armMotor"), hardwareMap.servo.get("yoinker"), hardwareMap.analogInput.get("potentiometer"));
        camera = new Camera(hardwareMap);

        camera.activate();

        Pose2d startPose = new Pose2d(-63, 0, Math.toRadians(0));

        drive.setPoseEstimate(startPose);



        //scan rings here 0, 1, 4 = A, B, C
        int rings = 0;

        for(int i = 0; i < 5; i++) {
            int curRings = 0;
            long startTime = System.currentTimeMillis();
            long endTime = System.currentTimeMillis();
            while(curRings == 0 && (endTime - startTime)/1000.0 < 1) {
                curRings = camera.checkTFODObjects(telemetry);
                endTime = System.currentTimeMillis();
            }
            rings = Math.max(rings, curRings);
        }

        telemetry.addData("Number of Rings: ", rings);
        telemetry.update();


        Trajectory startToShoot = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-2, 11.5, Math.toRadians(2)),
                        new MecanumConstraints(new DriveConstraints(
                                40, 30, 0.0,
                                Math.toRadians(180.0), Math.toRadians(180.0), 0.0), 13.9))
                .addTemporalMarker(1.5, () -> {
                    arm.armOut();
                })
                .build();

        Trajectory strafeLeftShoot = drive.trajectoryBuilder(startToShoot.end())
                .lineTo(new Vector2d(-2, 19),
                        new MecanumConstraints(new DriveConstraints(
                                30, 30, 0.0,
                                Math.toRadians(180.0), Math.toRadians(180.0), 0.0), 13.9))
                .build();

        Trajectory strafeleftShoot2 = drive.trajectoryBuilder(strafeLeftShoot.end())
                .lineTo(new Vector2d(-2, 26.5),
                        new MecanumConstraints(new DriveConstraints(
                                30, 30, 0.0,
                                Math.toRadians(180.0), Math.toRadians(180.0), 0.0), 13.9))
                .build();

        Trajectory downToWobble = drive.trajectoryBuilder(new Pose2d(-48, 0, Math.toRadians(0)))
                .lineTo(new Vector2d(-48, -11),
                        new MecanumConstraints(new DriveConstraints(
                                30, 30, 0.0,
                                Math.toRadians(180.0), Math.toRadians(180.0), 0.0), 13.9))
                .build();



        Trajectory shootToZoneA = drive.trajectoryBuilder(/*strafeleftShoot2.end()*/ new Pose2d(-2, 11.5, Math.toRadians(15)))
                .lineToLinearHeading(new Pose2d(14, -20, Math.toRadians(0)))
                .build();

        Trajectory zoneAToDown = drive.trajectoryBuilder(shootToZoneA.end(), true)
                .splineTo(new Vector2d(-48, 0), Math.toRadians(180))
                .build();

        Trajectory wobbleToZoneA = drive.trajectoryBuilder(downToWobble.end())
                .splineTo(new Vector2d(4, -20), Math.toRadians(0))
                .build();

        Trajectory zoneAToPark = drive.trajectoryBuilder(wobbleToZoneA.end())
                .lineToLinearHeading(new Pose2d(12, 0, Math.toRadians(0)))
                .addTemporalMarker(0.5, () -> {
                    arm.grab();
                })
                .build();



        Trajectory shootToZoneB = drive.trajectoryBuilder(/*strafeleftShoot2.end()*/new Pose2d(-2, 11.5, Math.toRadians(15)))
                .lineToLinearHeading(new Pose2d(36, 4, Math.toRadians(0)))
                .build();

        Trajectory zoneBToDown = drive.trajectoryBuilder(shootToZoneB.end(), true)
                .splineToConstantHeading(new Vector2d(-20, 6), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-48, 0), Math.toRadians(180))
                .build();

        Trajectory wobbleToRing = drive.trajectoryBuilder(downToWobble.end())
                .lineToLinearHeading(new Pose2d(-28, -14, Math.toRadians(7)),
                        new MecanumConstraints(new DriveConstraints(
                                16, 20, 0.0,
                                Math.toRadians(180.0), Math.toRadians(180.0), 0.0), 13.9))
                .build();

        Trajectory ringToZoneB = drive.trajectoryBuilder(wobbleToRing.end())
                .splineTo(new Vector2d(26, 4), Math.toRadians(0))
                .addTemporalMarker(0.5, () -> {
                    shooter.stopShooter();
                    intake.nosucc();
                    intake.stopRing();
                })
                .build();

        Trajectory zoneBToPark = drive.trajectoryBuilder(ringToZoneB.end(), true)
                .splineTo(new Vector2d(12, 4), Math.toRadians(180))
                .addTemporalMarker(0.5, () -> {
                    arm.grab();
                })
                .build();


        Trajectory startToShootC = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-40, -10, Math.toRadians(5)),
                        new MecanumConstraints(new DriveConstraints(
                                35, 30, 0.0,
                                Math.toRadians(180.0), Math.toRadians(180.0), 0.0), 13.9))
                .addTemporalMarker(1.0, () -> {
                    arm.armOut();
                })
                .build();

        Trajectory shootCToRing = drive.trajectoryBuilder(startToShootC.end())
                .lineToLinearHeading(new Pose2d(-24, -13, Math.toRadians(3)),
                        new MecanumConstraints(new DriveConstraints(
                                50, 60, 0.0,
                                Math.toRadians(180.0), Math.toRadians(180.0), 0.0), 13.65))
                .build();

        Trajectory backToCollect = drive.trajectoryBuilder(shootCToRing.end())
                .lineToLinearHeading(new Pose2d(-11, -12, Math.toRadians(5)),
                        new MecanumConstraints(new DriveConstraints(
                                3.2, 15, 0.0,
                                Math.toRadians(180.0), Math.toRadians(180.0), 0.0), 13.65))
                .addTemporalMarker(3, () -> {
                    intake.pushRing();
                })
                .build();

        Trajectory collectToZoneC = drive.trajectoryBuilder(backToCollect.end())
                .splineToConstantHeading(new Vector2d(60, -20), Math.toRadians(0),
                        new MecanumConstraints(new DriveConstraints(
                                50, 35, 0.0,
                                Math.toRadians(180.0), Math.toRadians(180.0), 0.0), 13.9))
                .addTemporalMarker(0.5, () -> {
                    intake.stopRing();
                    shooter.stopShooter();
                })
                .build();

        Trajectory zoneCToDown = drive.trajectoryBuilder(collectToZoneC.end(), true)
                .splineToConstantHeading(new Vector2d(-48, 0), Math.toRadians(180),
                        new MecanumConstraints(new DriveConstraints(
                                50, 35, 0.0,
                                Math.toRadians(180.0), Math.toRadians(180.0), 0.0), 13.9))
                .build();

        Trajectory wobbleToZoneC = drive.trajectoryBuilder(downToWobble.end())
                .splineToConstantHeading(new Vector2d(50, -20), Math.toRadians(0),
                        new MecanumConstraints(new DriveConstraints(
                                50, 35, 0.0,
                                Math.toRadians(180.0), Math.toRadians(180.0), 0.0), 13.9))
                .build();

        Trajectory zoneCToPark = drive.trajectoryBuilder(wobbleToZoneC.end())
                .lineTo(new Vector2d(9, -20),
                        new MecanumConstraints(new DriveConstraints(
                                50, 35, 0.0,
                                Math.toRadians(180.0), Math.toRadians(180.0), 0.0), 13.9))
                .addTemporalMarker(0.5, () -> {
                    arm.grab();
                })
                .build();

//        Trajectory shootToZoneC = drive.trajectoryBuilder(/*strafeleftShoot2.end()*/new Pose2d(-2, 11.5, Math.toRadians(17)))
//                .lineToLinearHeading(new Pose2d(60, -20, Math.toRadians(0)),
//                        new MecanumConstraints(new DriveConstraints(
//                                48, 32.5, 0.0,
//                                Math.toRadians(180.0), Math.toRadians(180.0), 0.0), 13.9))
//                .build();
//
//        Trajectory zoneCToDown = drive.trajectoryBuilder(shootToZoneC.end(), true)
//                .splineToConstantHeading(new Vector2d(-20, 4), Math.toRadians(180),
//                        new MecanumConstraints(new DriveConstraints(
//                                50, 35, 0.0,
//                                Math.toRadians(180.0), Math.toRadians(180.0), 0.0), 13.9))
//                .splineToConstantHeading(new Vector2d(-48, 0), Math.toRadians(180),
//                        new MecanumConstraints(new DriveConstraints(
//                                50, 35, 0.0,
//                                Math.toRadians(180.0), Math.toRadians(180.0), 0.0), 13.9))
//                .build();
//
//        Trajectory wobbleToRingC = drive.trajectoryBuilder(downToWobble.end())
//                .lineToLinearHeading(new Pose2d(-28, -14, Math.toRadians(6)))
//                .build();
//
//        Trajectory ringCToCollect = drive.trajectoryBuilder(wobbleToRingC.end())
//                .lineToLinearHeading(new Pose2d(-18, -14, Math.toRadians(6)),
//                        new MecanumConstraints(new DriveConstraints(
//                                3.5, 15, 0.0,
//                                Math.toRadians(180.0), Math.toRadians(180.0), 0.0), 13.65))
//                .addTemporalMarker(2.5, () -> {
//                    intake.pushRing();
//                })
//                .build();
//
//        Trajectory ringToRing2 = drive.trajectoryBuilder(wobbleToRingC.end())
//                .lineToLinearHeading(new Pose2d(-20, -14, Math.toRadians(6)),
//                        new MecanumConstraints(new DriveConstraints(
//                                30, 30, 0.0,
//                                Math.toRadians(180.0), Math.toRadians(180.0), 0.0), 13.9))
//                .build();
//
//        Trajectory ring2ToZoneC = drive.trajectoryBuilder(ringCToCollect.end())
//                .lineToLinearHeading(new Pose2d(50, -20, Math.toRadians(0)),
//                        new MecanumConstraints(new DriveConstraints(
//                                50, 35, 0.0,
//                                Math.toRadians(180.0), Math.toRadians(180.0), 0.0), 13.9))
//                .addTemporalMarker(1.5, () -> {
//                    shooter.stopShooter();
//                    intake.nosucc();
//                    intake.stopRing();
//                })
//                .build();
//
//        Trajectory zoneCToPark = drive.trajectoryBuilder(ring2ToZoneC.end())
//                .lineTo(new Vector2d(9, -20),
//                        new MecanumConstraints(new DriveConstraints(
//                                50, 35, 0.0,
//                                Math.toRadians(180.0), Math.toRadians(180.0), 0.0), 13.9))
//                .addTemporalMarker(0.5, () -> {
//                    arm.grab();
//                })
//                .build();


        arm.grab();

        waitForStart();

        if(isStopRequested()) return;

        if(rings != 4) {
            shooter.shootAuto();
            drive.followTrajectory(startToShoot);
            intake.pushRing();
            drive.residentSleeper(1250);
            intake.stopRing();
            //        drive.followTrajectory(strafeLeftShoot);
            drive.turn(Math.toRadians(9));
            intake.pushRing();
            drive.residentSleeper(1250);
            intake.stopRing();
            //        drive.followTrajectory(strafeleftShoot2);
            drive.turn(Math.toRadians(6));
            intake.pushRing();
            drive.residentSleeper(1100);
            intake.stopRing();
            shooter.stopShooter();
        }
        else {
            shooter.shootFar(0.63);
            drive.followTrajectory(startToShootC);
            intake.pushRing();
            drive.residentSleeper(600);
            shooter.shootFar(0.65);
            drive.residentSleeper(3100);
            intake.stopRing();
        }

        if(rings == 0) {
            drive.followTrajectory(shootToZoneA);
            drive.residentSleeper(250);
            arm.armDrop();
            drive.residentSleeper(500);
            arm.release();
            drive.residentSleeper(250);
            arm.armUp();
            drive.residentSleeper(250);
            drive.followTrajectory(zoneAToDown);
            arm.armDrop();
            drive.followTrajectory(downToWobble);
            drive.residentSleeper(500);
            arm.grab();
            drive.residentSleeper(750);
            arm.armOut();
            drive.residentSleeper(250);
            drive.followTrajectory(wobbleToZoneA);
            drive.residentSleeper(250);
            arm.armDrop();
            drive.residentSleeper(500);
            arm.release();
            drive.residentSleeper(250);
            arm.armRest();
            drive.followTrajectory(zoneAToPark);

        }
        else if(rings == 1) {
            drive.followTrajectory(shootToZoneB);
            drive.residentSleeper(100);
            arm.armDrop();
            drive.residentSleeper(500);
            arm.release();
            drive.residentSleeper(250);
            arm.armOut();
            drive.residentSleeper(250);
            drive.followTrajectory(zoneBToDown);
            arm.armDrop();
            drive.followTrajectory(downToWobble);
            drive.residentSleeper(300);
            arm.grab();
            drive.residentSleeper(700);
            arm.armOut();
            drive.residentSleeper(250);
            shooter.shootFar(0.64);
            intake.succ();
            drive.followTrajectory(wobbleToRing);
            drive.residentSleeper(1000);
            intake.pushRing();
            drive.residentSleeper(1000);
            drive.followTrajectory(ringToZoneB);
            drive.residentSleeper(100);
            arm.armDrop();
            drive.residentSleeper(500);
            arm.release();
            drive.residentSleeper(250);
            arm.armRest();
            drive.followTrajectory(zoneBToPark);
        }
        else if(rings == 4) {
            drive.followTrajectory(shootCToRing);
            intake.autoSucc(0.8);
            shooter.shootFar(0.67);
            drive.followTrajectory(backToCollect);
            drive.residentSleeper(2500);
            intake.nosucc();
            drive.followTrajectory(collectToZoneC);
            arm.armDrop();
            drive.residentSleeper(250);
            arm.release();
            drive.residentSleeper(100);
            arm.armOut();
            drive.residentSleeper(150);
            drive.followTrajectory(zoneCToDown);
            arm.armDrop();
            drive.followTrajectory(downToWobble);
            arm.grab();
            drive.residentSleeper(600);
            arm.armOut();
            drive.residentSleeper(50);
            drive.followTrajectory(wobbleToZoneC);
            arm.armDrop();
            drive.residentSleeper(250);
            arm.release();
            drive.residentSleeper(100);
            arm.armRest();
            drive.residentSleeper(150);
            drive.followTrajectory(zoneCToPark);

//            drive.followTrajectory(shootToZoneC);
//            arm.armDrop();
//            drive.residentSleeper(200);
//            arm.release();
//            drive.residentSleeper(100);
//            arm.armUp();
//            drive.residentSleeper(200);
//            drive.followTrajectory(zoneCToDown);
//            arm.armDrop();
//            drive.followTrajectory(downToWobble);
//            arm.grab();
//            drive.residentSleeper(500);
//            arm.armOut();
//            shooter.shoot();
//            drive.followTrajectory(wobbleToRingC);
//            intake.succ();
//            drive.followTrajectory(ringCToCollect);
//            //drive.residentSleeper(2500);
//            //drive.followTrajectory(ringToRing2);
//            drive.residentSleeper(2500);
//            drive.followTrajectory(ring2ToZoneC);
//            arm.armDrop();
//            drive.residentSleeper(200);
//            arm.release();
//            drive.residentSleeper(100);
//            arm.armRest();
//            drive.residentSleeper(200);
//            drive.followTrajectory(zoneCToPark);
        }
        PoseStorage.currentPose = drive.getPoseEstimate();
    }
}
