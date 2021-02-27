package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm {
    public DcMotor armMotor;
    public Servo armServo;
    Telemetry telemetry;

    private int restPosition = -5;
    private int upPosition = -50;  //100 = 30 degrees --> 3.33 = 1 degree
    private int outPosition = -425;
    private int downPosition = -525;
    private int dropPosition = -640;
    private int hitRingPosition = -750;
    private boolean clawOpen = true;
    private long cooldownTime = 500; //500 milliseconds
    private long grabbedTime = System.currentTimeMillis();

    public Arm(DcMotor armMotor, Servo armServo) {
        this.armMotor = armMotor;
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setPower(0.0);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.armServo = armServo;
    }

    public void controls(Gamepad gp) {
        long timeSinceGrab = System.currentTimeMillis() - grabbedTime;

        if(gp.a) {
            armUp();
        }
        else if(gp.b) {
            armOut();
        }
        else if(gp.y) {
            armDown();
        }
        else if(gp.dpad_up) {
            armRest();
        }

//        if(gp.right_bumper) {
//            grab();
//        }
//        else if(gp.left_bumper) {
//            release();
//        }

        if(gp.x && timeSinceGrab >= cooldownTime) {
            grabbedTime = System.currentTimeMillis();
            if(clawOpen) {
                grab();
                clawOpen = false;
            }
            else {
                release();
                clawOpen = true;
            }
        }
    }

    public void armRest() {
        armMotor.setTargetPosition(restPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.4);
    }

    public void armUp() {
        armMotor.setTargetPosition(upPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.4);
    }

    public void armDown() {
        armMotor.setTargetPosition(downPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.4);
    }

    public void armDrop() {
        armMotor.setTargetPosition(dropPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.4);
    }

    public void armHitRing() {
        armMotor.setTargetPosition(hitRingPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.4);
    }

    public void armOut() {
        armMotor.setTargetPosition(outPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.4);
}

    public void grab() {
        armServo.setPosition(0.725);
    }

    public void release() {
        armServo.setPosition(0.1);
    }

    public double testServo() {
        return armServo.getPosition();
    }

    public double testArm() {
        return armMotor.getCurrentPosition();
    }
}
