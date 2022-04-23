package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.robotcore.hardware.Servo;

public class GimbalMotion {
    private static final double YAW_ZERO = 0.5;
    private static final double PITCH_ZERO = 0.5;
    private final Servo yawServo;
    private final Servo pitchServo;

    public GimbalMotion(Servo y, Servo p) {
        yawServo = y;
        pitchServo = p;
    }

    public void resetPosition() {
        yawServo.setPosition(YAW_ZERO);
        pitchServo.setPosition(PITCH_ZERO);
    }

    public void setPosition(double n) {
        yawServo.setPosition(n);
        pitchServo.setPosition(n);
    }

}
