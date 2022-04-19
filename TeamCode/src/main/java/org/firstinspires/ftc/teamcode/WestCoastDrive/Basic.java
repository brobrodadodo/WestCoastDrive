package org.firstinspires.ftc.teamcode.WestCoastDrive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp (name = "Basic WCD", group = "Test")
public class Basic extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotor leftDrive1 = hardwareMap.dcMotor.get("leftDrive1");
        DcMotor leftDrive2 = hardwareMap.dcMotor.get("leftDrive2");
        DcMotor rightDrive1 = hardwareMap.dcMotor.get("rightDrive1");
        DcMotor rightDrive2 = hardwareMap.dcMotor.get("rightDrive2");

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double left = -gamepad1.left_stick_y;
            double right = gamepad1.right_stick_y;

            telemetry.addData("Right ", right);
            telemetry.addData("Left ", left);
            telemetry.update();

            leftDrive1.setPower(left);
            leftDrive2.setPower(left);
            rightDrive1.setPower(right);
            rightDrive2.setPower(right);
        }
    }
}
