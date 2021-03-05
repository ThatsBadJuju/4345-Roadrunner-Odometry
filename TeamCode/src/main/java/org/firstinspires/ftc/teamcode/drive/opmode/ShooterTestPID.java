package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import static org.firstinspires.ftc.teamcode.drive.opmode.TuningController.MOTOR_GEAR_RATIO;
import static org.firstinspires.ftc.teamcode.drive.opmode.TuningController.MOTOR_TICKS_PER_REV;


@TeleOp
public class ShooterTestPID extends LinearOpMode {
    // Copy your PIDF Coefficients here
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(50, 0, 10, 11.9);

    @Override
    public void runOpMode() throws InterruptedException {
        // SETUP MOTOR //
        // Change my id
        DcMotorEx myMotor = hardwareMap.get(DcMotorEx.class, "shooter");

        // Reverse as appropriate
        myMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Turns on bulk reading
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // RUE limits max motor speed to 85% by default
        // Raise that limit to 100%
        MotorConfigurationType motorConfigurationType = myMotor.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        myMotor.setMotorType(motorConfigurationType);

        // Turn on RUN_USING_ENCODER
        myMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set PIDF Coefficients with voltage compensated feedforward value
        myMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                MOTOR_VELO_PID.p, MOTOR_VELO_PID.i, MOTOR_VELO_PID.d,
                MOTOR_VELO_PID.f * 12 / hardwareMap.voltageSensor.iterator().next().getVoltage()
        ));

        // Insert whatever other initialization stuff you do here
        //4000 = power shot, 4175 = auto
        double targetVelocity = rpmToTicksPerSecond(5150);

        long time = System.currentTimeMillis();
        long cooldownTime = 350;

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            myMotor.setVelocity(targetVelocity);

            double motorVelo = myMotor.getVelocity();
            telemetry.addData("target", targetVelocity);
            telemetry.addData("velocity", motorVelo);
            telemetry.addData("error", targetVelocity - motorVelo);

            long timeSinceChange = System.currentTimeMillis() - time;
            if(timeSinceChange >= cooldownTime) {
                if(gamepad1.a) {
                    time = System.currentTimeMillis();
                    targetVelocity -= rpmToTicksPerSecond(250);
                }
                else if(gamepad1.b) {
                    time = System.currentTimeMillis();
                    targetVelocity += rpmToTicksPerSecond(25);
                }
                else if(gamepad1.y) {
                    time = System.currentTimeMillis();
                    targetVelocity += rpmToTicksPerSecond(250);
                }
                else if(gamepad1.x) {
                    time = System.currentTimeMillis();
                    targetVelocity -= rpmToTicksPerSecond(25);
                }

            }
            telemetry.update();
        }
    }

    public static double rpmToTicksPerSecond(double rpm) {
        return rpm * MOTOR_TICKS_PER_REV / MOTOR_GEAR_RATIO / 60;
    }
}