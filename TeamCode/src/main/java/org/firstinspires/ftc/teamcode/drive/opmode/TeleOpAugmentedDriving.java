package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.Arm;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.Intake;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.drive.Shooter;

/**
 * This opmode demonstrates how one can augment driver control by following Road Runner arbitrary
 * Road Runner trajectories at any time during teleop. This really isn't recommended at all. This is
 * not what Trajectories are meant for. A path follower is more suited for this scenario. This
 * sample primarily serves as a demo showcasing Road Runner's capabilities.
 * <p>
 * This bot starts in driver controlled mode by default. The player is able to drive the bot around
 * like any teleop opmode. However, if one of the select buttons are pressed, the bot will switch
 * to automatic control and run to specified location on its own.
 * <p>
 * If A is pressed, the bot will generate a splineTo() trajectory on the fly and follow it to
 * targetA (x: 45, y: 45, heading: 90deg).
 * <p>
 * If B is pressed, the bot will generate a lineTo() trajectory on the fly and follow it to
 * targetB (x: -15, y: 25, heading: whatever the heading is when you press B).
 * <p>
 * If Y is pressed, the bot will turn to face 45 degrees, no matter its position on the field.
 * <p>
 * Pressing X will cancel trajectory following and switch control to the driver. The bot will also
 * cede control to the driver once trajectory following is done.
 * <p>
 * The following may be a little off with this method as the trajectory follower and turn
 * function assume the bot starts at rest.
 * <p>
 * This sample utilizes the SampleMecanumDriveCancelable.java class.
 */
@TeleOp(group = "advanced")
public class TeleOpAugmentedDriving extends LinearOpMode {
    // Define 2 states, drive control or automatic control
    enum Mode {
        DRIVER_CONTROL,
        FIELD_CENTRIC,
        ALIGN_TO_POINT,
        AUTOMATIC_CONTROL
    }

    Mode currentMode = Mode.DRIVER_CONTROL;

    private PIDFController headingController = new PIDFController(SampleMecanumDrive.HEADING_PID);

    // The coordinates we want the bot to automatically go to when we press the A button
    Vector2d targetAVector = new Vector2d(0, 5);
    // The heading we want the bot to end on for targetA
    double targetAHeading = Math.toRadians(90);

    // The location we want the bot to automatically go to when we press the B button
    private Pose2d drivePosition = new Pose2d(-16, 0, Math.toRadians(0));

    // The angle we want to align to when we press Y
    double targetAngle = Math.toRadians(45);

    private Vector2d targetPosition = new Vector2d(72, 0);

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize custom cancelable SampleMecanumDrive class
        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);
        Intake intake = new Intake(/*hardwareMap.dcMotor.get("intakeMotor"), */hardwareMap.crservo.get("legsOfDoom"));
        Shooter shooter = new Shooter(hardwareMap.dcMotor.get("shooter"));
        Arm arm = new Arm(hardwareMap.dcMotor.get("armMotor"), hardwareMap.servo.get("yoinker"));

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        if(PoseStorage.currentPose.getX() == 0.0 && PoseStorage.currentPose.getY() == 0.0 &&
                PoseStorage.currentPose.getHeading() == 0.0) {
            drive.setPoseEstimate(new Pose2d(-63, 0, Math.toRadians(0))/*.plus(new Pose2d(0, 0, Math.toRadians(-90)))*/);
        }
        else drive.setPoseEstimate(PoseStorage.currentPose);

        headingController.setInputBounds(-Math.PI, Math.PI);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            // Update the drive class
            drive.update();

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            Pose2d driveDirection = new Pose2d();

            // Print pose to telemetry
            telemetry.addData("mode", currentMode);
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();

            // We follow different logic based on whether we are in manual driver control or switch
            // control to the automatic mode
            switch (currentMode) {
                case DRIVER_CONTROL:
                    if(gamepad1.left_trigger >= 0.05 || gamepad1.right_trigger >= 0.05) {
                        drive.setWeightedDrivePower(
                                new Pose2d(
                                        -gamepad1.left_stick_y * 0.75 / 3,
                                        -gamepad1.left_stick_x * 0.6 / 3,
                                        -gamepad1.right_stick_x * 0.75 / 3
                                )
                        );
                    }
                    else {
                        drive.setWeightedDrivePower(
                                new Pose2d(
                                        -gamepad1.left_stick_y * 0.75,
                                        -gamepad1.left_stick_x * 0.6,
                                        -gamepad1.right_stick_x * 0.75
                                )
                        );
                    }

                  if (gamepad1.left_stick_button) {
                        // If the B button is pressed on gamepad1, we generate a lineTo()
                        // trajectory on the fly and follow it
                        // We switch the state to AUTOMATIC_CONTROL

                        Trajectory traj1 = drive.trajectoryBuilder(poseEstimate)
                                .lineToLinearHeading(drivePosition)
                                .build();

                        drive.followTrajectoryAsync(traj1);

                        currentMode = Mode.AUTOMATIC_CONTROL;
                    }

//                  if(gamepad1.b) {
//                      currentMode = Mode.FIELD_CENTRIC;
//                  }
//                  if(gamepad1.y) {
//                      currentMode = Mode.ALIGN_TO_POINT;
//                  }
                    break;

//                case FIELD_CENTRIC:
//                    // Switch into alignment mode if `a` is pressed
//                    if (gamepad2.a) {
//                        currentMode = Mode.DRIVER_CONTROL;
//                    }
//                    if (gamepad2.y) {
//                        currentMode = Mode.ALIGN_TO_POINT;
//                    }
//
//
//
//                    // Standard teleop control
//                    // Convert gamepad input into desired pose velocity
//                    Vector2d input = new Vector2d(
//                            -gamepad1.left_stick_y,
//                            -gamepad1.left_stick_x
//                    ).rotated(-poseEstimate.getHeading() + Math.toRadians(90));
//
//                    // Pass in the rotated input + right stick value for rotation
//                    // Rotation is not part of the rotated input thus must be passed in separately
//                    drive.setWeightedDrivePower(
//                            new Pose2d(
//                                    input.getX(),
//                                    input.getY(),
//                                    -gamepad1.right_stick_x
//                            )
//                    );
//
//                    if (gamepad1.x) {
//                        // If the X button is pressed on gamepad2, we generate a lineTo()
//                        // trajectory on the fly and follow it
//                        // We switch the state to AUTOMATIC_CONTROL
//
//                        Trajectory traj1 = drive.trajectoryBuilder(poseEstimate)
//                                .lineToLinearHeading(drivePosition)
//                                .build();
//
//                        drive.followTrajectoryAsync(traj1);
//
//                        currentMode = Mode.AUTOMATIC_CONTROL;
//                    }
//                    break;
//
//                case ALIGN_TO_POINT:
//                    // Switch back into normal driver control mode if `b` is pressed
//                    if (gamepad1.a) {
//                        currentMode = Mode.DRIVER_CONTROL;
//                    }
//                    if(gamepad2.b) {
//                        currentMode = Mode.FIELD_CENTRIC;
//                    }
//
//                    // Create a vector from the gamepad x/y inputs which is the field relative movement
//                    // Then, rotate that vector by the inverse of that heading for field centric control
//                    Vector2d fieldFrameInput = new Vector2d(
//                            -gamepad1.left_stick_y,
//                            -gamepad1.left_stick_x
//                    );
//                    Vector2d robotFrameInput = fieldFrameInput.rotated(-poseEstimate.getHeading());
//
//                    // Difference between the target vector and the bot's position
//                    Vector2d difference = targetPosition.minus(poseEstimate.vec());
//                    // Obtain the target angle for feedback and derivative for feedforward
//                    double theta = difference.angle();
//
//                    // Not technically omega because its power. This is the derivative of atan2
//                    double thetaFF = -fieldFrameInput.rotated(-Math.PI / 2).dot(difference) / (difference.norm() * difference.norm());
//
//                    // Set the target heading for the heading controller to our desired angle
//                    headingController.setTargetPosition(theta);
//
//                    // Set desired angular velocity to the heading controller output + angular
//                    // velocity feedforward
//                    double headingInput = (headingController.update(poseEstimate.getHeading())
//                            * DriveConstants.kV + thetaFF)
//                            * DriveConstants.TRACK_WIDTH;
//
//                    // Combine the field centric x/y velocity with our derived angular velocity
//                    driveDirection = new Pose2d(
//                            robotFrameInput,
//                            headingInput
//                    );
//
//                    drive.setWeightedDrivePower(driveDirection);
//                    break;

                case AUTOMATIC_CONTROL:
                    // If x is pressed, we break out of the automatic following
                    if (gamepad1.right_stick_button) {
                        drive.cancelFollowing();
                        currentMode = Mode.DRIVER_CONTROL;
                    }

                    // If drive finishes its task, cede control to the driver
                    if (!drive.isBusy()) {
                        currentMode = Mode.DRIVER_CONTROL;
                    }
                    break;
            }
            intake.controls(gamepad1);
            shooter.controls(gamepad1);
            arm.controls(gamepad1);
        }
    }
}
