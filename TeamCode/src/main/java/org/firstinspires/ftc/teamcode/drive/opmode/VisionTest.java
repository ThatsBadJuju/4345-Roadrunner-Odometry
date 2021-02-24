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
        int rings = 0;
        rings = camera.checkTFODObjects(telemetry);
        telemetry.update();
        // telemetry.addData("Number of Rings: ", rings);
//        long startTime = System.currentTimeMillis();
//        long endTime = System.currentTimeMillis();
//        while(rings == 0 && (endTime - startTime)/1000.0 < 2) {
//            rings = camera.checkTFODObjects(telemetry);
//            endTime = System.currentTimeMillis();
//        }
//        telemetry.addData("Number of Rings: ", rings);
//        telemetry.addData("current time: ", endTime);
//        telemetry.update();
//        try {
//            Thread.sleep(10000);
//        } catch (InterruptedException e) {
//            e.printStackTrace();
//        }
    }

    @Override
    public void stop() {
        super.stop();
        camera.stop();
        telemetry.addLine("Robot stopped, camera off");
    }

}
