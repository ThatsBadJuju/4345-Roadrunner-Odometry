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

//        intake = new Intake(hardwareMap.dcMotor.get("intakeMotor"), hardwareMap.servo.get("useless"), hardwareMap.crservo.get("legsOfDoom"));
//        shooter = new Shooter(hardwareMap.dcMotor.get("shooter"));
//        arm = new Arm(hardwareMap.dcMotor.get("armMotor"), hardwareMap.servo.get("yoinker"));
//        camera = new Camera(hardwareMap);
//
//        camera.activate();

        Pose2d startPose = new Pose2d(-63, 0, Math.toRadians(0));

        drive.setPoseEstimate(startPose);



        //scan rings here 0, 1, 4 = A, B, C
        int rings = 4;

//        long startTime = System.currentTimeMillis();
//        long endTime = System.currentTimeMillis();
//        while(rings == 0 && (endTime - startTime)/1000.0 < 3.5) {
//            rings = camera.checkTFODObjects(telemetry);
//            endTime = System.currentTimeMillis();
//        }
//        telemetry.addData("Number of Rings: ", rings);
//        telemetry.update();


        Trajectory startToShoot = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(-2, 11.5))
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

        Trajectory downToWobble = drive.trajectoryBuilder(new Pose2d(-52, 0, Math.toRadians(0)))
                .lineTo(new Vector2d(-52, -11),
                        new MecanumConstraints(new DriveConstraints(
                                30, 30, 0.0,
                                Math.toRadians(180.0), Math.toRadians(180.0), 0.0), 13.9))
                .build();

        Trajectory wobbleToRing = drive.trajectoryBuilder(downToWobble.end())
                .lineTo(new Vector2d(-30, -11),
                        new MecanumConstraints(new DriveConstraints(
                                30, 30, 0.0,
                                Math.toRadians(180.0), Math.toRadians(180.0), 0.0), 13.9))
                .build();



        Trajectory shootToZoneA = drive.trajectoryBuilder(strafeleftShoot2.end())
                .splineTo(new Vector2d(26, -24), Math.toRadians(300))
                .build();

        Trajectory zoneAToDown = drive.trajectoryBuilder(shootToZoneA.end(), true)
                .splineTo(new Vector2d(-52, 0), Math.toRadians(180))
                .build();

        Trajectory wobbleToZoneA = drive.trajectoryBuilder(downToWobble.end())
                .splineTo(new Vector2d(8, -20), Math.toRadians(345))
                .build();



        Trajectory shootToZoneB = drive.trajectoryBuilder(strafeleftShoot2.end())
                .lineTo(new Vector2d(36, 0))
                .build();

        Trajectory zoneBToDown = drive.trajectoryBuilder(shootToZoneB.end(), true)
                .splineTo(new Vector2d(-52, 0), Math.toRadians(180))
                .build();

        Trajectory ringToZoneB = drive.trajectoryBuilder(wobbleToRing.end())
                .splineTo(new Vector2d(26, 0), Math.toRadians(0))
                .build();

        Trajectory zoneBToPark = drive.trajectoryBuilder(ringToZoneB.end(), true)
                .splineTo(new Vector2d(9, 0), Math.toRadians(180))
                .build();



        Trajectory shootToZoneC = drive.trajectoryBuilder(strafeleftShoot2.end())
                .lineTo(new Vector2d(60, -24))
                .build();

        Trajectory zoneCToDown = drive.trajectoryBuilder(shootToZoneC.end(), true)
                .splineTo(new Vector2d(0, -4), Math.toRadians(165))
                .splineTo(new Vector2d(-52, 0), Math.toRadians(180))
                .build();

        Trajectory ringToRing2 = drive.trajectoryBuilder(wobbleToRing.end())
                .lineTo(new Vector2d(-26, -11),
                        new MecanumConstraints(new DriveConstraints(
                                30, 30, 0.0,
                                Math.toRadians(180.0), Math.toRadians(180.0), 0.0), 13.9))
                .build();

        Trajectory ring2ToZoneC = drive.trajectoryBuilder(ringToRing2.end())
                .splineTo(new Vector2d(50, -24), Math.toRadians(0))
                .build();

        Trajectory zoneCToPark = drive.trajectoryBuilder(ring2ToZoneC.end())
                .lineTo(new Vector2d(9, -24))
                .build();



        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(startToShoot);
        drive.followTrajectory(strafeLeftShoot);
        drive.followTrajectory(strafeleftShoot2);

        if(rings == 0) {
            drive.followTrajectory(shootToZoneA);
            drive.followTrajectory(zoneAToDown);
            drive.followTrajectory(downToWobble);
            drive.followTrajectory(wobbleToZoneA);
        }
        else if(rings == 1) {
            drive.followTrajectory(shootToZoneB);
            drive.followTrajectory(zoneBToDown);
            drive.followTrajectory(downToWobble);
            drive.followTrajectory(wobbleToRing);
            drive.followTrajectory(ringToZoneB);
            drive.followTrajectory(zoneBToPark);
        }
        else if(rings == 4) {
            drive.followTrajectory(shootToZoneC);
            drive.followTrajectory(zoneCToDown);
            drive.followTrajectory(downToWobble);
            drive.followTrajectory(wobbleToRing);
            drive.followTrajectory(ringToRing2);
            drive.followTrajectory(ring2ToZoneC);
            drive.followTrajectory(zoneCToPark);
        }
    }
}
