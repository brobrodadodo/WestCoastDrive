package org.firstinspires.ftc.teamcode.WestCoastDrive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import me.wobblyyyy.pathfinder2.Pathfinder;
import me.wobblyyyy.pathfinder2.geometry.Translation;
import me.wobblyyyy.pathfinder2.kinematics.Kinematics;
import me.wobblyyyy.pathfinder2.kinematics.TankKinematics;
import me.wobblyyyy.pathfinder2.kinematics.TankState;
import me.wobblyyyy.pathfinder2.robot.Drive;
import me.wobblyyyy.pathfinder2.robot.ImprovedAbstractDrive;
import me.wobblyyyy.pathfinder2.robot.Odometry;
import me.wobblyyyy.pathfinder2.robot.Robot;
import me.wobblyyyy.pathfinder2.robot.components.Motor;
import me.wobblyyyy.pathfinder2.robot.components.MultiMotor;
import me.wobblyyyy.pathfinder2.robot.simulated.SimulatedOdometry;

@TeleOp(name = "TankDrive", group = "default")
public class TankDrive extends LinearOpMode {
    private Motor rightMotor1;
    private Motor rightMotor2;
    private Motor rightMotors;

    private Motor leftMotor1;
    private Motor leftMotor2;
    private Motor leftMotors;

    private Kinematics<TankState> kinematics;
    private Drive drive;
    private Odometry odometry;
    private Robot robot;
    private Pathfinder pathfinder;

    private void initialize() {
        DcMotor rightMotorDc1 =
                hardwareMap.get(DcMotor.class, "rightDrive1");
        DcMotor rightMotorDc2 =
                hardwareMap.get(DcMotor.class, "rightDrive2");
        rightMotor1 = new MotorWrapper(rightMotorDc1);
        rightMotor2 = new MotorWrapper(rightMotorDc2);
        rightMotors = new MultiMotor(rightMotor1, rightMotor2);

        DcMotor leftMotorDc1 =
                hardwareMap.get(DcMotor.class, "leftDrive1");
        DcMotor leftMotorDc2 =
                hardwareMap.get(DcMotor.class, "leftDrive2");
        leftMotor1 = new MotorWrapper(leftMotorDc1);
        leftMotor2 = new MotorWrapper(leftMotorDc2);
        leftMotors = new MultiMotor(leftMotor1, leftMotor2);

        kinematics = new TankKinematics(20);
        drive = new me.wobblyyyy.pathfinder2.drive.TankDrive(
            rightMotors,
            leftMotors,
            kinematics
        );
        odometry = new SimulatedOdometry();
        robot = new Robot(drive, odometry);
        pathfinder = new Pathfinder(robot, -0.05);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();

        pathfinder.onTick(() -> {
            drive.setTranslation(new Translation(
                gamepad1.left_stick_x,
                -gamepad1.left_stick_y,
                gamepad1.right_stick_x
            ));
        });

        while (opModeIsActive()) {
            pathfinder.tick();
        }
    }
}
