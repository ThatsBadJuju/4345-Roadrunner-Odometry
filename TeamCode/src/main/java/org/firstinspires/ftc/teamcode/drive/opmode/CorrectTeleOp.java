package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.Drivetrain;


@TeleOp(name = "TeleOp", group = "jerW" )
public class CorrectTeleOp extends OpMode {
    //TODO: Create instance variable for drivetrain
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
    }

    @Override
    public void stop() {
        super.stop();
        telemetry.addLine("Robot stopped");

    }
}

