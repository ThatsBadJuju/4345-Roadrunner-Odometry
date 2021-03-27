package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Intake {
    private DcMotor intakeMotor;
    private Servo transfer;

    private long transferTime = System.currentTimeMillis();

    public Intake(DcMotor intakeMotor, Servo transfer) {
        this.intakeMotor = intakeMotor;
        this.transfer = transfer;
        transfer.setPosition(0.5);
    }

    public void controls(Gamepad gp) {
        long timeSinceTransfer = System.currentTimeMillis() - transferTime;
        if(gp.left_bumper) {
            unsucc();
        }
        else if(gp.right_bumper) {
            succ();
        }
        else {
            nosucc();
        }

        if(gp.dpad_left) {
            reverseRing();
        }
        else if(gp.dpad_right) {
            pushRing();
        }
        else if(gp.x && timeSinceTransfer >= 500) {
            pushRingCycle(3);
        }
//        else stopRing();
    }

    public void succ() {
        intakeMotor.setPower(-0.8);
    }

    public void autoSucc(double power) {
        intakeMotor.setPower(-power);
    }

    public void unsucc() {
        intakeMotor.setPower(0.3);
    }

    public void nosucc() {
        intakeMotor.setPower(0.0);
    }


    public void pushRing() {
        transfer.setPosition(0.67);
        //crServo.setPower(1.0);
    }

    public void reverseRing() {
        transfer.setPosition(0.51);
        //crServo.setPower(-0.5);
    }

    public void pushRingCycle(int cycles) {
        long startTime = System.currentTimeMillis();
        long elapsedTime = System.currentTimeMillis();
        while(cycles > 0) {
            pushRing();
            while (elapsedTime - startTime <= 350) {
                elapsedTime = System.currentTimeMillis();
            }
            startTime = System.currentTimeMillis();
            reverseRing();
            while (elapsedTime - startTime <= 350) {
                elapsedTime = System.currentTimeMillis();
            }
            startTime = System.currentTimeMillis();
            cycles--;
        }
    }

    public void stopRing() {
        //crServo.setPower(0.0);
    }
}
