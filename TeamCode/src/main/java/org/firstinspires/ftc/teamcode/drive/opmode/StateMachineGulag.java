package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.ArmNoEncoder;
import org.firstinspires.ftc.teamcode.drive.Camera;
import org.firstinspires.ftc.teamcode.drive.Intake;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.Shooter;

import static org.firstinspires.ftc.teamcode.drive.opmode.TuningController.rpmToTicksPerSecond;

/**
 * This opmode explains how you follow multiple trajectories in succession, asynchronously. This
 * allows you to run your own logic beside the drive.update() command. This enables one to run
 * their own loops in the background such as a PID controller for a lift. We can also continuously
 * write our pose to PoseStorage.
 * <p>
 * The use of a State enum and a currentState field constitutes a "finite state machine."
 * You should understand the basics of what a state machine is prior to reading this opmode. A good
 * explanation can be found here:
 * https://www.youtube.com/watch?v=Pu7PMN5NGkQ (A finite state machine introduction tailored to FTC)
 * or here:
 * https://gm0.org/en/stable/docs/software/finite-state-machines.html (gm0's article on FSM's)
 * <p>
 * You can expand upon the FSM concept and take advantage of command based programming, subsystems,
 * state charts (for cyclical and strongly enforced states), etc. There is still a lot to do
 * to supercharge your code. This can be much cleaner by abstracting many of these things. This
 * opmode only serves as an initial starting point.
 */
@Autonomous(name = "PIDRightArmShoot", group = "advanced")
public class StateMachineGulag extends LinearOpMode {

    // This enum defines our "state"
    // This is essentially just defines the possible steps our program will take
    enum State {
        START_TO_SHOOT,
        SHOOT_TURN_1,
        SHOOT_TURN_2,
        DOWN_TO_WOBBLE,

        SHOOT_TO_ZONE_A,
        ZONE_A_TO_DOWN,
        WOBBLE_TO_ZONE_A,
        ZONE_A_TO_PARK,

        SHOOT_TO_ZONE_B,
        ZONE_B_TO_DOWN,
        WOBBLE_TO_RING,
        RING_TO_ZONE_B,
        ZONE_B_TO_PARK,

        START_TO_SHOOT_C,
        SHOOT_C_TO_RING,
        RING_TO_COLLECT,
        COLLECT_TO_ZONE_C,
        ZONE_C_TO_DOWN,
        WOBBLE_TO_ZONE_C,
        ZONE_C_TO_PARK,

        WAIT_100,
        WAIT_250,
        WAIT_500,
        WAIT_1000,

        IDLE            // Our bot will enter the IDLE state when done
    }

    public Intake intake;
    public Shooter shooter;
    public ArmNoEncoder arm;
    public Camera camera;

    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(50, 0, 10, 11.9);
    double highGoalVelo = rpmToTicksPerSecond(5150);
    double farGoalVelo = rpmToTicksPerSecond(4400);
    double powerShotVelo = rpmToTicksPerSecond(4000);

    boolean isFarGoal = false;
    boolean isPowerShot = false;

    ElapsedTime time = new ElapsedTime();

    // We define the current state we're on
    // Default to IDLE
    State currentState = State.IDLE;


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        intake = new Intake(hardwareMap.dcMotor.get("intakeMotor"), hardwareMap.crservo.get("legsOfDoom"));
        //shooter = new Shooter(hardwareMap.dcMotor.get("shooter"));
        arm = new ArmNoEncoder(hardwareMap.dcMotor.get("armMotor"), hardwareMap.servo.get("yoinker"), hardwareMap.analogInput.get("potentiometer"));
        camera = new Camera(hardwareMap);

        camera.activate();

        Pose2d startPose = new Pose2d(-63, 0, Math.toRadians(0));

        drive.setPoseEstimate(startPose);


        //_________________________________________________________________________________//

        // SETUP MOTOR //
        // Change my id
        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        // Reverse as appropriate
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        // Turns on bulk reading
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // RUE limits max motor speed to 85% by default
        // Raise that limit to 100%
        MotorConfigurationType motorConfigurationType = shooter.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        shooter.setMotorType(motorConfigurationType);

        // Turn on RUN_USING_ENCODER
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set PIDF Coefficients with voltage compensated feedforward value
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                MOTOR_VELO_PID.p, MOTOR_VELO_PID.i, MOTOR_VELO_PID.d,
                MOTOR_VELO_PID.f * 12 / hardwareMap.voltageSensor.iterator().next().getVoltage()
        ));


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


        Trajectory downToWobble = drive.trajectoryBuilder(new Pose2d(-48, 0, Math.toRadians(0)))
                .lineTo(new Vector2d(-48, -11),
                        new MecanumConstraints(new DriveConstraints(
                                30, 30, 0.0,
                                Math.toRadians(180.0), Math.toRadians(180.0), 0.0), 13.9))
                .build();



        Trajectory shootToZoneA = drive.trajectoryBuilder(/*strafeleftShoot2.end()*/ new Pose2d(-2, 11.5, Math.toRadians(17)))
                .lineToLinearHeading(new Pose2d(14, -20, Math.toRadians(0)))
                .build();

        double shootTurn1 = Math.toRadians(9);

        double shootTurn2 = Math.toRadians(6);

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



        Trajectory shootToZoneB = drive.trajectoryBuilder(/*strafeleftShoot2.end()*/new Pose2d(-2, 11.5, Math.toRadians(17)))
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
                    isFarGoal = false;
                    isPowerShot = false;
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

        Trajectory ringToCollect = drive.trajectoryBuilder(shootCToRing.end())
                .lineToLinearHeading(new Pose2d(-11, -12, Math.toRadians(5)),
                        new MecanumConstraints(new DriveConstraints(
                                3.2, 15, 0.0,
                                Math.toRadians(180.0), Math.toRadians(180.0), 0.0), 13.65))
                .addTemporalMarker(3, () -> {
                    intake.pushRing();
                })
                .build();

        Trajectory collectToZoneC = drive.trajectoryBuilder(ringToCollect.end())
                .splineToConstantHeading(new Vector2d(60, -20), Math.toRadians(0),
                        new MecanumConstraints(new DriveConstraints(
                                50, 35, 0.0,
                                Math.toRadians(180.0), Math.toRadians(180.0), 0.0), 13.9))
                .addTemporalMarker(0.5, () -> {
                    intake.stopRing();
                    isFarGoal = false;
                    isPowerShot = false;
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


        arm.grab();

        waitForStart();

        if (isStopRequested()) return;

        // Set the current state to TRAJECTORY_1, our first step
        // Then have it follow that trajectory
        // Make sure you use the async version of the commands
        // Otherwise it will be blocking and pause the program here until the trajectory finishes
        if(rings != 4) {
            currentState = State.START_TO_SHOOT;
            drive.followTrajectoryAsync(startToShoot);
        }


        while (opModeIsActive() && !isStopRequested()) {
            // Our state machine logic
            // You can have multiple switch statements running together for multiple state machines
            // in parallel. This is the basic idea for subsystems and commands.

            // We essentially define the flow of the state machine through this switch statement
            switch (currentState) {
                case START_TO_SHOOT:
                    // Check if the drive class isn't busy
                    // `isBusy() == true` while it's following the trajectory
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished
                    // We move on to the next state
                    // Make sure we use the async follow function
                    if(drive.isBusy()) {
                        time.reset();
                    }
                    if (!drive.isBusy() && time.milliseconds() >= 500) {
                        currentState = State.SHOOT_TURN_1;
                        drive.turnAsync(shootTurn1);
                    }
                    break;
                case SHOOT_TURN_1:
                    // Check if the drive class isn't busy
                    // `isBusy() == true` while it's following the trajectory
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished
                    // We move on to the next state
                    // Make sure we use the async follow function
                    if (!drive.isBusy()) {
                        currentState = State.SHOOT_TURN_2;
                        drive.turnAsync(shootTurn2);
                    }
                    break;
                case SHOOT_TURN_2:
                    // Check if the drive class is busy following the trajectory
                    // Move on to the next state, TURN_1, once finished
                    if (!drive.isBusy() && rings == 0) {
                        currentState = State.SHOOT_TO_ZONE_A;
                        drive.followTrajectoryAsync(shootToZoneA);
                    }
                    else if(!drive.isBusy() && rings == 1) {
                        currentState = State.SHOOT_TO_ZONE_B;
                        drive.followTrajectoryAsync(shootToZoneB);
                    }
                    break;


                case SHOOT_TO_ZONE_A:
                    // Check if the drive class is busy turning
                    // If not, move onto the next state, TRAJECTORY_3, once finished
                    if (!drive.isBusy()) {
                        currentState = State.SHOOT_TO_ZONE_B;
                        drive.followTrajectoryAsync(shootToZoneB);
                    }
                    break;

//                case :
//                    // Check if the drive class is busy following the trajectory
//                    // If not, move onto the next state, WAIT_1
//                    if (!drive.isBusy()) {
//                        currentState = State.WAIT_1;
//
//                        // Start the wait timer once we switch to the next state
//                        // This is so we can track how long we've been in the WAIT_1 state
//                        waitTimer1.reset();
//                    }
//                    break;
//                case WAIT_1:
//                    // Check if the timer has exceeded the specified wait time
//                    // If so, move on to the TURN_2 state
//                    if (waitTimer1.seconds() >= waitTime1) {
//                        currentState = State.TURN_2;
//                        drive.turnAsync(turnAngle2);
//                    }
//                    break;
//                case TURN_2:
//                    // Check if the drive class is busy turning
//                    // If not, move onto the next state, IDLE
//                    // We are done with the program
//                    if (!drive.isBusy()) {
//                        currentState = State.IDLE;
//                    }
//                    break;

                case WAIT_100:

                case IDLE:
                    // Do nothing in IDLE
                    // currentState does not change once in IDLE
                    // This concludes the autonomous program
                    break;
            }

            // Anything outside of the switch statement will run independent of the currentState

            // We update drive continuously in the background, regardless of state
            drive.update();
            // We update our lift PID continuously in the background, regardless of state
            if(isFarGoal) {
                shooter.setVelocity(farGoalVelo);
            }
            else if(isPowerShot) {
                shooter.setVelocity(powerShotVelo);
            }
            else shooter.setVelocity(0);

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Continually write pose to `PoseStorage`
            PoseStorage.currentPose = poseEstimate;

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }

    public void PIDSleeper(int ms) {

    }
}