package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TeleOp", group="TeleOp")
public class TeleOpMode extends OpMode {
    private DcMotorEx leftFront, rightFront, leftRear, rightRear;
    private DcMotor hex_motor;
    private double theta;

    @Override
    public void init() {
        // Initialize motors using the hardwareMap
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
//        hex_motor = hardwareMap.get(DcMotor.class, "hex_motor");

        // Set motor directions if needed
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        // Controller inputs
        double driveX = gamepad1.left_stick_x;  // X-axis movement
        double driveY = -gamepad1.left_stick_y; // Y-axis movement (negative due to reversed Y-axis)
        double turn = gamepad1.right_stick_x;   // Turning component
//        double hex_up = -gamepad1.right_stick_y; // Control for hex motor

        // Calculate magnitude and direction
        double magnitude = Math.hypot(driveX, driveY);
        theta = Math.atan2(driveY, driveX);

        // Calculate wheel power components
        double sin = Math.sin(theta - Math.PI / 4);
        double cos = Math.cos(theta - Math.PI / 4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        // Calculate motor powers
        double leftFrontPower = (magnitude * cos / max) + turn;
        double rightFrontPower = (magnitude * sin / max) - turn;
        double leftRearPower = (magnitude * sin / max) + turn;
        double rightRearPower = (magnitude * cos / max) - turn;

        // Set motor powers with clipping to ensure they remain in the range [-1.0, 1.0]
        leftFront.setPower(Range.clip(leftFrontPower, -1.0, 1.0)*0.7);
        rightFront.setPower(Range.clip(rightFrontPower, -1.0, 1.0)*0.7);
        leftRear.setPower(Range.clip(leftRearPower, -1.0, 1.0)*0.7);
        rightRear.setPower(Range.clip(rightRearPower, -1.0, 1.0)*0.7);
//        hex_motor.setPower(Range.clip(hex_up, -1.0, 1.0));
    }
}