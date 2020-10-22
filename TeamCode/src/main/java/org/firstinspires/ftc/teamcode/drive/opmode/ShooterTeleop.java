package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.Shooter;

@TeleOp(name = "Shooter", group = "4345-tests" )
public class ShooterTeleop extends OpMode {
    public Shooter shooter;

    @Override
    public void init() {
        shooter = new Shooter(hardwareMap.dcMotor.get("shooter"));
    }

    @Override
    public void loop() {
        shooter.controls(gamepad1);
    }
}
