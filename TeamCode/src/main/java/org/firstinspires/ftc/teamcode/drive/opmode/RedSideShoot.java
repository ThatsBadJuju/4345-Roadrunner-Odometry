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

@Autonomous(name = "WobbleVisionShoot", group = "drive" )
public class RedSideShoot extends LinearOpMode {

    public Intake intake;
    public Shooter shooter;
    public Arm arm;
    public Camera camera;

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        intake = new Intake(hardwareMap.dcMotor.get("intakeMotor"), hardwareMap.servo.get("useless"), hardwareMap.crservo.get("legsOfDoom"));
        shooter = new Shooter(hardwareMap.dcMotor.get("shooter"));
        arm = new Arm(hardwareMap.dcMotor.get("armMotor"), hardwareMap.servo.get("yoinker"));
        camera = new Camera(hardwareMap);

        camera.activate();

        Pose2d startPose = new Pose2d(-63, -24, Math.toRadians(0));

        drive.setPoseEstimate(startPose);



        //scan rings here 0, 1, 4 = A, B, C
        int rings = 0;

        long startTime = System.currentTimeMillis();
        long endTime = System.currentTimeMillis();
        while(rings == 0 && (endTime - startTime)/1000.0 < 3) {
            rings = camera.checkTFODObjects(telemetry);
            endTime = System.currentTimeMillis();
        }
        telemetry.addData("Number of Rings: ", rings);
        telemetry.update();

        Trajectory toZoneA = drive.trajectoryBuilder(startPose)
                .forward(46)
                .splineTo(new Vector2d(7, -33), Math.toRadians(305))
                .build();


        Trajectory zoneAToDown = drive.trajectoryBuilder(toZoneA.end(), true)
                .splineTo(new Vector2d(-15, -24), Math.toRadians(180))
                .back(37)
                .build();


        Trajectory downToZoneA = drive.trajectoryBuilder(zoneAToDown.end())
                .forward(26)
                .splineTo(new Vector2d(-2, -33), Math.toRadians(305))
                .build();

        Trajectory zoneABackUp = drive.trajectoryBuilder(downToZoneA.end())
                .lineTo(new Vector2d(-8, -33))
                .build();

        Trajectory zoneAToShoot = drive.trajectoryBuilder(zoneABackUp.end())
                .lineToLinearHeading(new Pose2d(-2, 14, Math.toRadians(5.5)))
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


        Trajectory zoneBBackUp = drive.trajectoryBuilder(downToZoneB.end())
                .back(8)
                .build();


        Trajectory zoneBToShoot = drive.trajectoryBuilder(zoneBBackUp.end())
                .lineToLinearHeading(new Pose2d(-2, 14, Math.toRadians(5.5)))
                .build();




        Trajectory toZoneC = drive.trajectoryBuilder(startPose)
                .forward(87)
                .splineTo(new Vector2d(48, -33), Math.toRadians(305))
                .build();

        Trajectory zoneCToDown = drive.trajectoryBuilder(toZoneC.end(), true)
                .splineTo(new Vector2d(24, -24), Math.toRadians(180))
                .back(76)
                .build();

        Trajectory downToZoneC = drive.trajectoryBuilder(zoneCToDown.end())
                .forward(67)
                .splineTo(new Vector2d(39, -33), Math.toRadians(305))
                .build();

        Trajectory zoneCToShoot = drive.trajectoryBuilder(downToZoneC.end())
                .lineToLinearHeading(new Pose2d(-2, 14, Math.toRadians(5.5)))
                .build();




        Trajectory downToWobble = drive.trajectoryBuilder(new Pose2d(-52, -24, Math.toRadians(0)), false)
                .lineTo(new Vector2d(-52, -12),
                        new MecanumConstraints(new DriveConstraints(
                                20, 20, 0.0,
                                Math.toRadians(180.0), Math.toRadians(180.0), 0.0), 13.9))
                .build();

        Trajectory wobbleToDown = drive.trajectoryBuilder(new Pose2d(-52, -12, Math.toRadians(0)), false)
                .strafeRight(12.0)
                .build();


        Trajectory shootToPark = drive.trajectoryBuilder(new Pose2d(-2, 14, Math.toRadians(6)))
                .forward(11)
                .build();



        arm.grab();

        waitForStart();

        if(isStopRequested()) return;



        if(rings == 0) {
            drive.followTrajectory(toZoneA);
            arm.armDown();                      //will turn into dropWobble method (cleanup)
            drive.residentSleeper(1000);
            arm.release();
            drive.residentSleeper(250);
            arm.armUp();
            drive.followTrajectory(zoneAToDown);
            arm.armDown();

            drive.followTrajectory(downToWobble);
            drive.residentSleeper(500);
            arm.grab();
            drive.residentSleeper(500);
            arm.armUp();
            drive.followTrajectory(wobbleToDown);

            drive.followTrajectory(downToZoneA);
            arm.armDown();
            drive.residentSleeper(1000);
            arm.release();
            arm.armRest();
            drive.residentSleeper(250);
            intake.liftIntake();
            drive.followTrajectory(zoneABackUp);
            shooter.shoot();
            drive.followTrajectory(zoneAToShoot);

            intake.pushRing();
            drive.residentSleeper(1000);
            intake.stopRing();
            drive.turn(Math.toRadians(-6)); //turns right?
            drive.residentSleeper(2000);
            intake.pushRing();
            drive.residentSleeper(1000);
            intake.stopRing();
            drive.turn(Math.toRadians(12.0));   //turns left?
            drive.residentSleeper(2000);
            intake.pushRing();
            drive.residentSleeper(1000);
            intake.stopRing();
            drive.followTrajectory(shootToPark);
        }
        else if(rings == 1) {
            drive.followTrajectory(toZoneB);
            arm.armDown();
            drive.residentSleeper(1000);
            arm.release();
            drive.residentSleeper(250);
            arm.armUp();
            drive.followTrajectory(zoneBToDown);
            arm.armDown();

            drive.followTrajectory(downToWobble);
            drive.residentSleeper(500);
            arm.grab();
            drive.residentSleeper(500);
            arm.armUp();
            drive.followTrajectory(wobbleToDown);

            drive.followTrajectory(downToZoneB);
            arm.armDown();
            drive.residentSleeper(1000);
            arm.release();
            arm.armRest();
            drive.residentSleeper(250);
            intake.liftIntake();
            drive.followTrajectory(zoneBBackUp);
            shooter.shoot();
            drive.followTrajectory(zoneBToShoot);

            intake.liftIntake();
            intake.pushRing();
            drive.residentSleeper(500);
            intake.stopRing();
            drive.turn(Math.toRadians(-5.5));
            drive.residentSleeper(1000);
            intake.pushRing();
            drive.residentSleeper(500);
            intake.stopRing();
            drive.turn(Math.toRadians(11.0));
            drive.residentSleeper(1000);
            intake.pushRing();
            drive.residentSleeper(500);
            intake.stopRing();
            drive.followTrajectory(shootToPark);
        }
        else if(rings == 4) {
            drive.followTrajectory(toZoneC);
            arm.armDown();
            drive.residentSleeper(1000);
            arm.release();
            drive.residentSleeper(250);
            arm.armUp();
            drive.followTrajectory(zoneCToDown);
            arm.armDown();

            drive.followTrajectory(downToWobble);
            drive.residentSleeper(500);
            arm.grab();
            drive.residentSleeper(500);
            arm.armUp();
            drive.followTrajectory(wobbleToDown);

            drive.followTrajectory(downToZoneC);
            arm.armDown();
            drive.residentSleeper(1000);
            arm.release();
            arm.armRest();
            drive.residentSleeper(250);
            intake.liftIntake();
            shooter.shoot();
            drive.followTrajectory(zoneCToShoot);

            intake.liftIntake();
            intake.pushRing();
            drive.residentSleeper(500);
            intake.stopRing();
            drive.turn(Math.toRadians(-5.5));
            drive.residentSleeper(1000);
            intake.pushRing();
            drive.residentSleeper(500);
            intake.stopRing();
            drive.turn(Math.toRadians(11.0));
            drive.residentSleeper(1000);
            intake.pushRing();
            drive.residentSleeper(500);
            intake.stopRing();
            drive.followTrajectory(shootToPark);
        }
    }
}
