package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ShooterTest {
    private DcMotor flywheel;
    private Telemetry telemetry;
    private double power;
    private long cooldownTime = 250; //250 milliseconds
    private long time = System.currentTimeMillis();

    public ShooterTest(DcMotor flywheel, Telemetry telemetry) {
        this.flywheel = flywheel;
        this.telemetry = telemetry;
        power = 0;
    }

    public void controls(Gamepad gp) {

    }
}