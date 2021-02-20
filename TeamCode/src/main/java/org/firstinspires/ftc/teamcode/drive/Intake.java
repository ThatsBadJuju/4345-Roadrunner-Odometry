package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    //private DcMotor intakeMotor;
    private CRServo crServo;

    public Intake(CRServo crServo) {
        //this.intakeMotor = intakeMotor;
        this.crServo = crServo;
    }

    public void controls(Gamepad gp) {
//        if(gp.left_bumper) {
//            unsucc();
//        }
//        else if(gp.right_bumper) {
//            succ();
//        }
//        else {
//            nosucc();
//        }

        if(gp.dpad_left) {
            reverseRing();
        }
        else if(gp.dpad_right) {
            pushRing();
        }
        else stopRing();
    }

//    public void succ() {
//        intakeMotor.setPower(-0.65);
//    }
//
//    public void unsucc() {
//        intakeMotor.setPower(0.3);
//    }
//
//    public void nosucc() {
//        intakeMotor.setPower(0.0);
//    }

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
