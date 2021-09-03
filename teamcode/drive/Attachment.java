package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Attachment {

    protected Servo servo = null;
    protected double upPosition = 0.0;
    //protected double middlePosition = 0.5;
    protected double downPosition = 0.0;

    public void init(HardwareMap hardwareMap, String servoName, double upValue, double downValue) {
        servo = hardwareMap.get(Servo.class, servoName);

        upPosition = upValue;
        //middlePosition = middleValue
        downPosition = downValue;
    }

    public void setServoForward() {
        servo.setDirection(Servo.Direction.FORWARD);
    }

    public void setServoReverse() {
        servo.setDirection(Servo.Direction.REVERSE);
    }

    public void up() {
        servo.setPosition(upPosition);
    }

    public void down() {
        servo.setPosition(downPosition);
    }

//    public void middle() { servo.setPosition(middlePosition); }

    public void toggle(boolean toggleState) {
        if(toggleState) {
            up();
        } else {
            down();
        }
    }

    public void setPosition(double position) {
        servo.setPosition(position);
    }


}