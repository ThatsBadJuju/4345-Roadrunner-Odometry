package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.drive.opmode.TuningController.rpmToTicksPerSecond;

public class VerticalIntake {
    private DcMotor intakeMotor;
    private Servo dropServo;
    private AnalogInput potentiometer;

    private boolean turnOffServo = false;
    private boolean isSucc = false;
    private boolean isUnsucc = false;
    private long time = System.currentTimeMillis();
    private long cooldownTime = 400;

    public VerticalIntake(DcMotor intakeMotor, Servo dropServo, boolean isAuto) {
        this.intakeMotor = intakeMotor;
        this.dropServo = dropServo;
        if(isAuto) {
            turnOffServo = false;
        }
        else {
            turnOffServo = true;
        }
    }

    public void controls(Gamepad gp) {
        long timeSinceChange = System.currentTimeMillis() - time;
        if (gp.left_trigger >= 0.05) {
            unsucc();
        } else if (gp.left_bumper) {
            succ();
        }

        if(timeSinceChange >= cooldownTime) {
            if(gp.right_bumper && isSucc) {
                time = System.currentTimeMillis();
                nosucc();
                isSucc = false;
                isUnsucc = false;
            }
            else if(gp.right_bumper) {
                time = System.currentTimeMillis();
                succ();
                isSucc = true;
                isUnsucc = false;
            }
            else if(gp.left_bumper && isUnsucc) {
                time = System.currentTimeMillis();
                nosucc();
                isSucc = false;
                isUnsucc = false;
            }
            else if(gp.left_bumper) {
                time = System.currentTimeMillis();
                unsucc();
                isSucc = false;
                isUnsucc = true;
            }
        }

        if(isSucc) {
            succ();
        }
        else if(isUnsucc) {
            unsucc();
        }
        else nosucc();


        if(gp.dpad_left) {
            pwmOff();
            turnOffServo = true;
        }
        else if(gp.dpad_right) {
            pwmOn();
            turnOffServo = false;
        }
        else if(gp.dpad_up) {
            drop();
        }

        if(turnOffServo = true) {
            pwmOff();
        }
        else if(turnOffServo = false) {
            pwmOn();
        }
//        else stopRing();
    }

    public void succ() {
        intakeMotor.setPower(-0.5);
    }


    public void unsucc() {
        intakeMotor.setPower(0.3);
    }

    public void nosucc() {
        intakeMotor.setPower(0.0);
    }

    public void drop() {
        dropServo.getController().pwmEnable();
        long startTime = System.currentTimeMillis();
        long elapsedTime = System.currentTimeMillis();
        dropServo.setPosition(1.0);
        while(elapsedTime - startTime <= 1500) {
            elapsedTime = System.currentTimeMillis();
        }
        dropServo.getController().pwmDisable();
        turnOffServo = true;
    }

    public void update() {
        if(turnOffServo) {
            pwmOff();
        }
        else {
            pwmOn();
        }
    }

    public void pwmOn() {
        dropServo.getController().pwmEnable();
    }

    public void pwmOff() {
        dropServo.getController().pwmDisable();
    }
}
