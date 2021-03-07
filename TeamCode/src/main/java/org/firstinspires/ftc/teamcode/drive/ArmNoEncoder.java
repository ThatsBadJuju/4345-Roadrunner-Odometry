package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ArmNoEncoder {
    public DcMotor armMotor;
    public Servo armServo;
    public AnalogInput potentiometer;
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
    private long armTime = System.currentTimeMillis();

    private boolean armRest = false;
    private boolean armUp = false;
    private boolean armGrab = true;

    private boolean armRestAuto = true;
    private boolean armOutAuto = false;
    private boolean armDropAuto = false;

    public ArmNoEncoder(DcMotor armMotor, Servo armServo, AnalogInput potentiometer) {
        this.armMotor = armMotor;
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setPower(0.0);
        this.armServo = armServo;
        this.potentiometer = potentiometer;
    }

    public void controls(Gamepad gp) {
        long timeSinceGrab = System.currentTimeMillis() - grabbedTime;
        long timeSinceArmChange = System.currentTimeMillis() - armTime;

        if(timeSinceArmChange >= cooldownTime) {
            if (gp.a) {
                armTime = System.currentTimeMillis();
                armRun(25, 0.4);
                armRest = true;
                armUp = false;
                armGrab = false;
            }
            else if (gp.b) {
                armTime = System.currentTimeMillis();
                armRun(60, 0.4);
                armRest = false;
                armUp = true;
                armGrab = false;
            }
            else if (gp.y) {
                armTime = System.currentTimeMillis();
                armRun(165, 0.4);
                armRest = false;
                armUp = false;
                armGrab = true;
            }
        }

        if(armRest) {
            armRun(25, 0.4);
        }
        else if(armUp) {
            armRun(110, 0.4);
        }
        else if(armGrab) {
            armRun(165, 0.4);
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

    public double getVoltage() {
        return potentiometer.getVoltage();
    }

    public void armRun(double angleTo, double maxPower) {
        double deltaAngle = angleTo - getAngle();
        double correctedPower = 0;
        if(deltaAngle >= 0) {
            correctedPower = -0.025 - deltaAngle * 0.015;
        }
        else correctedPower = 0.025 - deltaAngle * 0.015;

        correctedPower += Math.sin(Math.toRadians(getAngle() - 70)) * 0.075;

        if(correctedPower >= maxPower) {
            correctedPower = maxPower;
        }
        else if(correctedPower <= -maxPower) {
            correctedPower = -maxPower;
        }

        armMotor.setPower(correctedPower);
    }

    public void armRest() {
        armRun(25, 0.4);
        armRestAuto = true;
        armOutAuto = false;
        armDropAuto = false;

    }

    public void armOut() {
        armRun(160, 0.4);
        armRestAuto = false;
        armOutAuto = true;
        armDropAuto = false;
    }

    public void armDrop() {
        armRun(175, 0.4);
        armRestAuto = false;
        armOutAuto = false;
        armDropAuto = true;
    }

    public double getAngle() {
        double x = potentiometer.getVoltage();
        double sqrtFunction = Math.sqrt(21870000*x*x-24057000*x+19847025);
        return (2700*x + 4455 - sqrtFunction)/(20*x);
    }

    public void update() {
        if(armRestAuto) {
            armRest();
        }
        else if(armOutAuto) {
            armOut();
        }
        else if(armDropAuto) {
            armDrop();
        }
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
}
