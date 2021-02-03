package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm {
    public DcMotor armMotor;
    public Servo armServo;
    Telemetry telemetry;

    private int restPosition = -10;
    private int upPosition = -200;  //almost directly upwards (maybe closer to robot side by 10-15 degrees)
    private int downPosition = -825;
    private boolean clawOpen = false;

    public Arm(DcMotor armMotor, Servo armServo) {
        this.armMotor = armMotor;
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setPower(0.0);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.armServo = armServo;
    }

    public void controls(Gamepad gp) {
        if(gp.a) {
            armRest();
        }
        else if(gp.b) {
            armUp();
        }
        else if(gp.y) {
            armDown();
        }

        if(gp.right_bumper) {
            grab();
        }
        else if(gp.left_bumper) {
            release();
        }
    }

    public void armRest() {
        armMotor.setTargetPosition(restPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.2);
    }

    public void armUp() {
        armMotor.setTargetPosition(upPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.25);
    }

    public void armDown() {
        armMotor.setTargetPosition(downPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.2);
}

    public void grab() {
        armServo.setPosition(0.08);
    }

    public void release() {
        armServo.setPosition(0.9);
    }

    public double testServo() {
        return armServo.getPosition();
    }

    public double testArm() {
        return armMotor.getCurrentPosition();
    }
}
