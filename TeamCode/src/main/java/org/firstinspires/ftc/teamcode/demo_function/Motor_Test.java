package org.firstinspires.ftc.teamcode.demo_function;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Motor Ticks Test", group = "Test")
public class Motor_Test extends OpMode {

    // Khai báo động cơ
    private DcMotorEx motor_encode;

    @Override
    public void init() {
        // Khởi tạo động cơ từ hardwareMap
        motor_encode = hardwareMap.get(DcMotorEx.class, "motor_encode");  // Đảm bảo "motor_encode" là tên động cơ trong hardware map

        // Đặt chế độ của động cơ
        motor_encode.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER); // Chỉ reset encoder một lần khi khởi động
        motor_encode.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);     // Bắt đầu sử dụng encoder
    }

    @Override
    public void loop() {
        // Kiểm tra giá trị từ gamepad để điều khiển động cơ
        double motorPower = gamepad1.left_stick_y;  // Sử dụng joystick trái (trục Y) để điều khiển động cơ

        // Điều khiển động cơ bằng gamepad
        motor_encode.setPower(motorPower);

        // Hiển thị số ticks hiện tại của động cơ lên Driver Station
        telemetry.addData("Motor Power", motorPower);
        telemetry.addData("Motor Ticks", motor_encode.getCurrentPosition());
        telemetry.update();
    }
}
