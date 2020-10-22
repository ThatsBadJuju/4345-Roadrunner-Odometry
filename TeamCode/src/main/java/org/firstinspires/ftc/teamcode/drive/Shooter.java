package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Shooter {
    private DcMotor flywheel;
    private Telemetry telemetry;
    private double power;
    public Shooter(DcMotor flywheel, Telemetry telemetry) {
        this.flywheel = flywheel;
        this.telemetry = telemetry;
        power = 0;
    }

    public void controls(Gamepad gp) {
        if(gp.dpad_up) {
            increasePower(0.1);
        }
        else if(gp.dpad_down) {
            decreasePower(0.1);
        }
        else if(gp.dpad_right) {
            increasePower(0.01);
        }
        else if(gp.dpad_left) {
            decreasePower(0.01);
        }
        else if(gp.a) {
            start();
        }
        else if(gp.b) {
            stop();
        }
        else if(gp.y) {
            reset();
        }
    }

    public void increasePower(double adder) {
        if(power + adder <= 1) {
            power += adder;
        }
        else power = 1;
        flywheel.setPower(power);
    }

    public void decreasePower(double subtractor) {
        if(power - subtractor >= -1) {
            power -= subtractor;
        }
        else power = -1;
        flywheel.setPower(power);
    }

    public void stop() {
        flywheel.setPower(0);
    }

    public void start() {
        flywheel.setPower(power);
    }

    public void reset() {
        stop();
        power = 0;
    }

    public double getPower() {
        return power;
    }
}
