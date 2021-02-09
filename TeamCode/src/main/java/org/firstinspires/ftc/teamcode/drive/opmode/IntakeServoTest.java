package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.Arm;
import org.firstinspires.ftc.teamcode.drive.Camera;
import org.firstinspires.ftc.teamcode.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.drive.Intake;
import org.firstinspires.ftc.teamcode.drive.Shooter;


@TeleOp(name = "IntakeServoTest", group = "jerW" )
public class IntakeServoTest extends OpMode {
    //TODO: Create instance variable for drivetrain
    public Intake intake;

    @Override
    public void init() {
        //TODO: Initialize Drivetrain Object
        intake = new Intake(hardwareMap.dcMotor.get("intakeMotor"), hardwareMap.servo.get("useless"), hardwareMap.crservo.get("legsOfDoom"));
    }

    @Override
    public void start() {
        super.start();
        telemetry.addLine("Robot started");
    }

    @Override
    public void loop() {
        //TODO: Controls
        intake.controls(gamepad1);
        telemetry.addLine("Useless: " + String.valueOf(intake.testLift()));
    }

    @Override
    public void stop() {
        super.stop();
        telemetry.addLine("Robot stopped");

    }
}

