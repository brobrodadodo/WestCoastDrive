package org.firstinspires.ftc.teamcode.WestCoastDrive;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.function.Supplier;

import me.wobblyyyy.pathfinder2.Pathfinder;
import me.wobblyyyy.pathfinder2.geometry.Angle;
import me.wobblyyyy.pathfinder2.geometry.PointXYZ;
import me.wobblyyyy.pathfinder2.geometry.Translation;
import me.wobblyyyy.pathfinder2.kinematics.DifferentialDriveOdometry;
import me.wobblyyyy.pathfinder2.kinematics.EncoderConverter;
import me.wobblyyyy.pathfinder2.kinematics.EncoderTracker;
import me.wobblyyyy.pathfinder2.kinematics.Kinematics;
import me.wobblyyyy.pathfinder2.kinematics.TankKinematics;
import me.wobblyyyy.pathfinder2.kinematics.TankOdometry;
import me.wobblyyyy.pathfinder2.kinematics.TankState;
import me.wobblyyyy.pathfinder2.odometry.DifferentialOdometry;
import me.wobblyyyy.pathfinder2.odometry.TankDriveOdometry;
import me.wobblyyyy.pathfinder2.robot.Drive;
import me.wobblyyyy.pathfinder2.robot.Odometry;
import me.wobblyyyy.pathfinder2.robot.Robot;
import me.wobblyyyy.pathfinder2.robot.components.Motor;
import me.wobblyyyy.pathfinder2.robot.components.MultiMotor;
import me.wobblyyyy.pathfinder2.robot.simulated.SimulatedOdometry;
import me.wobblyyyy.pathfinder2.trajectory.LinearTrajectory;

// rightMotor1 and leftMotor1 have encoders
@TeleOp(name = "TankDrive", group = "default")
public class TankDrive extends LinearOpMode {
    private DcMotor rightMotorDc1;
    private DcMotor rightMotorDc2;
    private Motor rightMotor1;
    private Motor rightMotor2;
    private Motor rightMotors;

    private DcMotor leftMotorDc1;
    private DcMotor leftMotorDc2;
    private Motor leftMotor1;
    private Motor leftMotor2;
    private Motor leftMotors;

    private BNO055IMU imu;
    private IMUGyro gyro;

    private Kinematics<TankState> kinematics;
    private DifferentialOdometry odometry;
    private Drive drive;
    private Robot robot;
    private Pathfinder pathfinder;

    private void initialize() {
        rightMotorDc1 = hardwareMap.get(DcMotor.class, "rightDrive1");
        rightMotorDc2 = hardwareMap.get(DcMotor.class, "rightDrive2");
        rightMotor1 = new MotorWrapper(rightMotorDc1);
        rightMotor2 = new MotorWrapper(rightMotorDc2);
        rightMotors = new MultiMotor(rightMotor1, rightMotor2).invert();

        leftMotorDc1 = hardwareMap.get(DcMotor.class, "leftDrive1");
        leftMotorDc2 = hardwareMap.get(DcMotor.class, "leftDrive2");
        leftMotor1 = new MotorWrapper(leftMotorDc1);
        leftMotor2 = new MotorWrapper(leftMotorDc2);
        leftMotors = new MultiMotor(leftMotor1, leftMotor2);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        gyro = new IMUGyro(imu);
        gyro.setAngle(Angle.fromDeg(0));

        kinematics = new TankKinematics(-0.005, 20);
        EncoderConverter converter = new EncoderConverter(1_440, 4 * Math.PI);
        Supplier<Integer> rightTicks = rightMotorDc1::getCurrentPosition;
        Supplier<Integer> leftTicks = () -> -leftMotorDc1.getCurrentPosition();
        odometry = new DifferentialOdometry(
                new DifferentialDriveOdometry(Angle.ZERO, PointXYZ.ZERO),
                converter,
                rightTicks,
                leftTicks,
                gyro::getAngle
        );
        drive = new me.wobblyyyy.pathfinder2.drive.TankDrive(
                rightMotors,
                leftMotors,
                kinematics
        );
        robot = new Robot(drive, odometry);
        pathfinder = new Pathfinder(robot, -0.005);
        pathfinder.setAngleTolerance(Angle.fromDeg(5));
        pathfinder.setTolerance(4);
        pathfinder.setSpeed(0.25);
        odometry.offsetSoPositionIs(PointXYZ.ZERO);
    }

    @Override
    public void runOpMode() {
        initialize();

        waitForStart();

        /*
        pathfinder.onTick(() -> {
            drive.setTranslation(new Translation(
                    gamepad1.left_stick_x,
                    -gamepad1.left_stick_y,
                    gamepad1.right_stick_x
            ));
        });
        */

        pathfinder.onTick(() -> {
            telemetry.addData("gyro angle", gyro.getAngle());
            telemetry.addData("right ticks", rightMotorDc1.getCurrentPosition());
            telemetry.addData("left ticks", -leftMotorDc1.getCurrentPosition());
            telemetry.addData("position", pathfinder.getPosition());

            telemetry.update();
        });

//        pathfinder.lockHeading(Angle.fromDeg(45));
        pathfinder.goToY(32);
        pathfinder.goToY(0);
//        pathfinder.goToZ(new Angle(90));
//        pathfinder.goToZ(new Angle(135));
//        pathfinder.goToZ(new Angle(180));

        while (opModeIsActive()) {
            pathfinder.tick();
        }
    }
}
