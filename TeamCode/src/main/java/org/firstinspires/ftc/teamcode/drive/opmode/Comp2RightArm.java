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

        intake = new Intake(hardwareMap.crservo.get("legsOfDoom"));
        shooter = new Shooter(hardwareMap.dcMotor.get("shooter"));
        arm = new Arm(hardwareMap.dcMotor.get("armMotor"), hardwareMap.servo.get("yoinker"));
        camera = new Camera(hardwareMap);

        camera.activate();

        Pose2d startPose = new Pose2d(-63, 0, Math.toRadians(0));

        drive.setPoseEstimate(startPose);



        //scan rings here 0, 1, 4 = A, B, C
        int rings = 0;

        long startTime = System.currentTimeMillis();
        long endTime = System.currentTimeMillis();
        while(rings == 0 && (endTime - startTime)/1000.0 < 3.5) {
            rings = camera.checkTFODObjects(telemetry);
            endTime = System.currentTimeMillis();
        }
        telemetry.addData("Number of Rings: ", rings);
        telemetry.update();


        Trajectory startToShoot = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-2, 11.5, Math.toRadians(0)),
                        new MecanumConstraints(new DriveConstraints(
                                35, 30, 0.0,
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

        Trajectory wobbleToRing = drive.trajectoryBuilder(downToWobble.end())
                .lineToLinearHeading(new Pose2d(-30, -11, Math.toRadians(5)),
                        new MecanumConstraints(new DriveConstraints(
                                20, 20, 0.0,
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
                .splineTo(new Vector2d(-48, 0), Math.toRadians(180))
                .build();

        Trajectory ringToZoneB = drive.trajectoryBuilder(wobbleToRing.end())
                .splineTo(new Vector2d(26, 4), Math.toRadians(0))
                .build();

        Trajectory zoneBToPark = drive.trajectoryBuilder(ringToZoneB.end(), true)
                .splineTo(new Vector2d(12, 4), Math.toRadians(180))
                .addTemporalMarker(0.5, () -> {
                    arm.grab();
                })
                .build();



        Trajectory shootToZoneC = drive.trajectoryBuilder(/*strafeleftShoot2.end()*/new Pose2d(-2, 11.5, Math.toRadians(15)))
                .lineToLinearHeading(new Pose2d(60, -20, Math.toRadians(0)))
                .build();

        Trajectory zoneCToDown = drive.trajectoryBuilder(shootToZoneC.end(), true)
                .splineTo(new Vector2d(0, -4), Math.toRadians(165))
                .splineTo(new Vector2d(-48, 0), Math.toRadians(180))
                .build();

        Trajectory ringToRing2 = drive.trajectoryBuilder(wobbleToRing.end())
                .lineToLinearHeading(new Pose2d(-26, -11, Math.toRadians(5)),
                        new MecanumConstraints(new DriveConstraints(
                                30, 30, 0.0,
                                Math.toRadians(180.0), Math.toRadians(180.0), 0.0), 13.9))
                .build();

        Trajectory ring2ToZoneC = drive.trajectoryBuilder(ringToRing2.end())
                .lineToLinearHeading(new Pose2d(50, -20, Math.toRadians(0)))
                .build();

        Trajectory zoneCToPark = drive.trajectoryBuilder(ring2ToZoneC.end())
                .lineTo(new Vector2d(9, -20))
                .addTemporalMarker(0.5, () -> {
                    arm.grab();
                })
                .build();


        arm.grab();

        waitForStart();

        if(isStopRequested()) return;

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
        drive.residentSleeper(1250);
        intake.stopRing();
        shooter.stopShooter();

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
            drive.residentSleeper(500);
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
            drive.residentSleeper(250);
            arm.armDrop();
            drive.residentSleeper(500);
            arm.release();
            drive.residentSleeper(250);
            arm.armUp();
            drive.residentSleeper(250);
            drive.followTrajectory(zoneBToDown);
            arm.armDrop();
            drive.followTrajectory(downToWobble);
            drive.residentSleeper(500);
            arm.grab();
            drive.residentSleeper(500);
            arm.armOut();
            drive.residentSleeper(250);
            shooter.shoot();
            intake.pushRing();
            drive.followTrajectory(wobbleToRing);
            drive.residentSleeper(1500);
            shooter.stopShooter();
            intake.stopRing();
            drive.followTrajectory(ringToZoneB);
            drive.residentSleeper(250);
            arm.armDrop();
            drive.residentSleeper(500);
            arm.release();
            drive.residentSleeper(250);
            arm.armRest();
            drive.followTrajectory(zoneBToPark);
        }
        else if(rings == 4) {
            drive.followTrajectory(shootToZoneC);
            arm.armDrop();
            drive.residentSleeper(500);
            arm.release();
            drive.residentSleeper(250);
            arm.armUp();
            drive.residentSleeper(250);
            drive.followTrajectory(zoneCToDown);
            arm.armDrop();
            drive.followTrajectory(downToWobble);
            drive.residentSleeper(500);
            arm.grab();
            drive.residentSleeper(500);
            arm.armOut();
            drive.residentSleeper(250);
            shooter.shoot();
            intake.pushRing();
            drive.followTrajectory(wobbleToRing);
            drive.residentSleeper(1500);
            drive.followTrajectory(ringToRing2);
            drive.residentSleeper(1000);
            shooter.stopShooter();
            intake.stopRing();
            drive.followTrajectory(ring2ToZoneC);
            arm.armDrop();
            drive.residentSleeper(500);
            arm.release();
            drive.residentSleeper(250);
            arm.armRest();
            drive.residentSleeper(250);
            drive.followTrajectory(zoneCToPark);
        }
        PoseStorage.currentPose = drive.getPoseEstimate();
    }
}
