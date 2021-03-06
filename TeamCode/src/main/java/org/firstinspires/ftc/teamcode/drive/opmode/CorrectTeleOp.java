package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.Arm;
import org.firstinspires.ftc.teamcode.drive.ArmNoEncoder;
import org.firstinspires.ftc.teamcode.drive.Camera;
import org.firstinspires.ftc.teamcode.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.drive.Intake;
import org.firstinspires.ftc.teamcode.drive.Shooter;


@TeleOp(name = "TeleOp", group = "jerW" )
public class CorrectTeleOp extends OpMode {
    //TODO: Create instance variable for drivetrain
    public Drivetrain drivetrain;
    public Intake intake;
    public Shooter shooter;
    public ArmNoEncoder arm;
    // public Camera camera;

    @Override
    public void init() {
        //TODO: Initialize Drivetrain Object
        drivetrain = new Drivetrain(hardwareMap.dcMotor.get("frontLeftMotor"), hardwareMap.dcMotor.get("backLeftMotor"), hardwareMap.dcMotor.get("frontRightMotor"), hardwareMap.dcMotor.get("backRightMotor"), false, telemetry, hardwareMap);
        intake = new Intake(hardwareMap.dcMotor.get("intakeMotor"), hardwareMap.crservo.get("legsOfDoom"));
        shooter = new Shooter(hardwareMap.dcMotor.get("shooter"));
        arm = new ArmNoEncoder(hardwareMap.dcMotor.get("armMotor"), hardwareMap.servo.get("yoinker"), hardwareMap.analogInput.get("potentiometer"));
        // camera = new Camera(hardwareMap);
    }

    @Override
    public void start() {
        super.start();
        // camera.activate();
        telemetry.addLine("Robot started");
    }

    @Override
    public void loop() {
        //TODO: Controls
        drivetrain.controls(gamepad1);
        intake.controls(gamepad1);
        shooter.controls(gamepad1);
        arm.controls(gamepad1);
        //camera.checkVuforiaObjects(telemetry);
        telemetry.addLine("Yoinker: " + String.valueOf(arm.testServo()));
        //telemetry.addLine("Arm: " + String.valueOf(arm.testArm()));
        telemetry.addLine("Voltage: " + String.valueOf(arm.getVoltage()));
        telemetry.addLine("Angle: " + String.valueOf(arm.getAngle()));
    }

    @Override
    public void stop() {
        super.stop();
        // camera.stop();
        telemetry.addLine("Robot stopped");

    }
}

