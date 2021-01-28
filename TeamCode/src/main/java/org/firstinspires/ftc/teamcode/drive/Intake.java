package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    private Servo servo;
    private CRServo crServo;

    private double dropPosition = 0.5;
    private double liftPosition = 0.7;

    public Intake(Servo servo, CRServo crServo) {
        this.servo = servo;
        this.crServo = crServo;

    }

    public void controls(Gamepad gp) {
        if(gp.dpad_down) {
            //dropIntake();
        }
        else if(gp.dpad_up) {
            //liftIntake();
        }

        if(gp.dpad_left) {
            reverseRing();
        }
        else if(gp.dpad_right) {
            pushRing();
        }
        else stopRing();
    }

    public void dropIntake() {
        servo.setPosition(dropPosition);
    }

    public void liftIntake() {
        servo.setPosition(liftPosition);
    }

    public void pushRing() {
        crServo.setPower(0.5);
    }

    public void reverseRing() {
        crServo.setPower(-0.5);
    }

    public void stopRing() {
        crServo.setPower(0.0);
    }
}
