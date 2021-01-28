package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Shooter {
    private DcMotor shooter;

    public Shooter(DcMotor shooter) {
        this.shooter = shooter;
    }

    public void controls(Gamepad gp) {
        if(gp.left_bumper) {
            reverseShoot();
        }
        else if(gp.right_bumper) {
            shoot();
        }
        else stopShooter();
    }

    public void shoot() {
        shooter.setPower(-0.65);
    }

    public void reverseShoot() {
        shooter.setPower(0.1);
    }

    public void stopShooter() {
        shooter.setPower(0.01);
    }
}
