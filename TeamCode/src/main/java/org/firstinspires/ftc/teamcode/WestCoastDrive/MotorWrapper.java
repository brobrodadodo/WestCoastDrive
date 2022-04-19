package org.firstinspires.ftc.teamcode.WestCoastDrive;

import com.qualcomm.robotcore.hardware.DcMotor;
import me.wobblyyyy.pathfinder2.robot.components.BaseMotor;

public class MotorWrapper extends BaseMotor {
    private final DcMotor motor;

    public MotorWrapper(DcMotor motor) {
        this.motor = motor;
    }

    @Override
    public void abstractSetPower(double power) {
        motor.setPower(power);
    }
}
