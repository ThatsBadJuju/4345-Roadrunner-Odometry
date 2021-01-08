package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.Shooter;
import org.firstinspires.ftc.teamcode.drive.ShooterTest;

@TeleOp(name = "Shooter", group = "4345-tests" )
public class ShooterTeleop extends OpMode {
    public ShooterTest shooter;

    @Override
    public void init() {
        shooter = new ShooterTest(hardwareMap.dcMotor.get("shooter"), telemetry);
    }

    @Override
    public void loop() {
        shooter.controls(gamepad1);
        telemetry.addData("Current power", shooter.getPower());
        telemetry.update();
    }
}
