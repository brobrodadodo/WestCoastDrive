package org.firstinspires.ftc.teamcode.WestCoastDrive;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import me.wobblyyyy.pathfinder2.geometry.Angle;
import me.wobblyyyy.pathfinder2.robot.sensors.AbstractGyroscope;

public class IMUGyro extends AbstractGyroscope {
    private final BNO055IMU imu;

    public IMUGyro(BNO055IMU imu) {
        this.imu = imu;

        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        params.calibrationDataFile = "BNO055IMUCalibration.json";
        params.loggingEnabled = true;
        params.loggingTag = "IMU";
        params.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(params);
    }

    @Override
    public Angle getRawAngle() {
        Orientation angles = imu.getAngularOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.XYZ,
                AngleUnit.DEGREES
        );

        return Angle.fixedDeg(angles.thirdAngle);
    }
}
