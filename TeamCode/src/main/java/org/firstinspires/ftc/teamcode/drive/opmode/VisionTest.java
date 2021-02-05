package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.Camera;
import org.firstinspires.ftc.teamcode.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.drive.Intake;
import org.firstinspires.ftc.teamcode.drive.Shooter;

@TeleOp(name = "VisionTest", group = "jerW")
public class VisionTest extends OpMode {

    // public Drivetrain drivetrain;
    // public Intake intake;
    // public Shooter shooter;
    public Camera camera;

    @Override
    public void init() {
        // drivetrain = new Drivetrain(hardwareMap.dcMotor.get("frontLeftMotor"), hardwareMap.dcMotor.get("backLeftMotor"), hardwareMap.dcMotor.get("frontRightMotor"), hardwareMap.dcMotor.get("backRightMotor"), false, telemetry, hardwareMap);
        // intake = new Intake(hardwareMap.servo.get("useless"), hardwareMap.crservo.get("legsOfDoom"));
        // shooter = new Shooter(hardwareMap.dcMotor.get("shooter"));
        camera = new Camera(hardwareMap);
    }

    @Override
    public void start() {
        super.start();
        camera.activate();
        telemetry.addLine("Robot started, camera on");
    }

    @Override
    public void loop() {
        //TODO: Controls
        // drivetrain.controls(gamepad1);
        // intake.controls(gamepad1);
        // shooter.controls(gamepad1);
        telemetry.addData("Object recognized", camera.checkTFODObjects(telemetry));
        telemetry.update();
    }

    @Override
    public void stop() {
        super.stop();
        camera.stop();
        telemetry.addLine("Robot stopped, camera off");
    }

}
