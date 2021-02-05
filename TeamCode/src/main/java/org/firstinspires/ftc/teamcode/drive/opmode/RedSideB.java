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

@Autonomous(name = "DoubleWobbleParkB", group = "drive" )
public class RedSideB extends LinearOpMode {

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







        Trajectory downToWobble = drive.trajectoryBuilder(new Pose2d(-52, -24, Math.toRadians(0)), false)
                .lineTo(new Vector2d(-52, -12),
                        new MecanumConstraints(new DriveConstraints(
                                20, 20, 0.0,
                                Math.toRadians(180.0), Math.toRadians(180.0), 0.0), 13.9))
                .build();

        Trajectory wobbleToDown = drive.trajectoryBuilder(new Pose2d(-52, -12, Math.toRadians(0)), false)
                .strafeRight(12.0)
                .build();


        arm.grab();

        waitForStart();

        if(isStopRequested()) return;



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
        drive.residentSleeper(250);
        arm.armUp();
        drive.followTrajectory(zoneBToPark);
        arm.armRest();
        drive.residentSleeper(1000);
    }
}
