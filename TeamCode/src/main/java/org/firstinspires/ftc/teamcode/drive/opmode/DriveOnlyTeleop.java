package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.Arm;
import org.firstinspires.ftc.teamcode.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.drive.Intake;
import org.firstinspires.ftc.teamcode.drive.Shooter;

@TeleOp(name = "DriveOnly(Test)", group = "jerW" )
public class DriveOnlyTeleop extends OpMode {
    public Drivetrain drivetrain;

    @Override
    public void init() {
        //TODO: Initialize Drivetrain Object
        drivetrain = new Drivetrain(hardwareMap.dcMotor.get("frontLeftMotor"), hardwareMap.dcMotor.get("backLeftMotor"), hardwareMap.dcMotor.get("frontRightMotor"), hardwareMap.dcMotor.get("backRightMotor"), false, telemetry, hardwareMap);
    }

    @Override
    public void start() {
        super.start();
        telemetry.addLine("Robot started");
    }

    @Override
    public void loop() {
        //TODO: Controls
        drivetrain.controls(gamepad1);
        telemetry.addData("Current strafe coefficient", drivetrain.getStrafeCoef());
        telemetry.update();
    }

    @Override
    public void stop() {
        super.stop();
        telemetry.addLine("Robot stopped");

    }
}
