package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TeleOp", group="TeleOp")
public class TeleOpMode extends OpMode {
    private DcMotorEx leftFront, rightFront, leftRear, rightRear;
    private DcMotor hex_motor;
    private double theta;
    private Servo servo1, servo2, claw;
    private DcMotorEx motor_encode;

    @Override
    public void init() {
        // Initialize motors using the hardwareMap
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        servo1 = hardwareMap.get(Servo.class, "servo 1");
        servo2 = hardwareMap.get(Servo.class, "servo 2");
        claw = hardwareMap.get(Servo.class, "claw");
        motor_encode = hardwareMap.get(DcMotorEx.class, "motor encode");
//        hex_motor = hardwareMap.get(DcMotor.class, "hex_motor");

        // Set motor directions if needed
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        motor_encode.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER); // Chỉ reset encoder một lần khi khởi động
        motor_encode.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);     // Bắt đầu sử dụng encoder
    }

    @Override
    public void loop() {
        // Controller inputs
        double driveX = gamepad1.left_stick_x;  // X-axis movement
        double driveY = -gamepad1.left_stick_y; // Y-axis movement (negative due to reversed Y-axis)
        double turn = gamepad1.right_stick_x;   // Turning component
        double servo_position, claw_position, ticks;
        double motor_power;
        servo_position = servo1.getPosition();
        claw_position = claw.getPosition();
        motor_power=motor_encode.getCurrentPosition();


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
        leftFront.setPower(Range.clip(leftFrontPower, -1.0, 1.0) * 0.2);
        rightFront.setPower(Range.clip(rightFrontPower, -1.0, 1.0) * 0.2);
        leftRear.setPower(Range.clip(leftRearPower, -1.0, 1.0) * 0.2);
        rightRear.setPower(Range.clip(rightRearPower, -1.0, 1.0) * 0.2);
//        hex_motor.setPower(Range.clip(hex_up, -1.0, 1.0));

        if (gamepad1.dpad_up) {
            // Tăng dần vị trí servo (quay lên)
            motor_power += 100; // Điều chỉnh tốc độ bằng cách thay đổi giá trị này
//            if (motor_power > 1.0) {
//                motor_power = 1.0; // Giới hạn vị trí tối đa
//            }

        } else if (gamepad1.dpad_down) {
            // Giảm dần vị trí servo (quay xuống)
            motor_power -= 100; // Điều chỉnh tốc độ bằng cách thay đổi giá trị này
//            if (motor_power < 0.0) {
//                motor_power = 0.0; // Giới hạn vị trí tối thiểu
//            }
        if (gamepad2.dpad_up) {
            // Tăng dần vị trí servo (quay lên)
            servo_position += 0.01; // Điều chỉnh tốc độ bằng cách thay đổi giá trị này
            if (servo_position > 1.0) {
                servo_position = 1.0; // Giới hạn vị trí tối đa
            }
        } else if (gamepad2.dpad_down) {
            // Giảm dần vị trí servo (quay xuống)
            servo_position -= 0.01; // Điều chỉnh tốc độ bằng cách thay đổi giá trị này
            if (servo_position < 0.0) {
                servo_position = 0.0; // Giới hạn vị trí tối thiểu
            }
        }
        if (gamepad2.a) {
            claw_position += 0.01;
            if (claw_position > 1.0) {
                claw_position = 1.0;
            }
        }
        else if (gamepad2.b) {
            claw_position -= 0.01;
            if (claw_position < 0.0) {
                claw_position = 0;
            }

        }
        servo1.setPosition(servo_position);
        servo2.setPosition(servo_position);
        claw.setPosition(claw_position);
        motor_encode.setPower(motor_power);
        //telemetry
        }
    }
}
