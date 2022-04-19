package org.firstinspires.ftc.teamcode.WestCoastDrive;

import com.qualcomm.robotcore.hardware.DcMotor;
import me.wobblyyyy.pathfinder2.robot.sensors.AbstractEncoder;

public class EncoderWrapper extends AbstractEncoder {
    private final DcMotor motor;

    public EncoderWrapper(DcMotor motor) {
        this.motor = motor;
    }

    @Override
    public int getRawTicks() {
        return motor.getCurrentPosition();
    }
}
