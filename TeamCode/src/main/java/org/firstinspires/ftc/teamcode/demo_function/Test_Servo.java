package org.firstinspires.ftc.teamcode.demo_function;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Test Servo", group = "TeleOp")
public class Test_Servo extends LinearOpMode {

    // Khai báo servo
    private Servo Servo1;
    private Servo Servo2, Servo3;


    // Biến lưu vị trí hiện tại của servo
    private double servoPosition = 0; // Vị trí trung bình (giữa)
    private double servoPosition1;
    // Biến thời gian để điều khiển tốc độ servo
    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Khởi tạo servo
        Servo1 = hardwareMap.get(Servo.class, "servo 1");
        Servo2 = hardwareMap.get(Servo.class, "servo 2");
        Servo3= hardwareMap.get(Servo.class,"servo 3");
        servoPosition1=Servo3.getPosition();

        // Đặt vị trí ban đầu của servo
        Servo1.setPosition(servoPosition);
        Servo2.setPosition(servoPosition);
        Servo3.setPosition(servoPosition1);
        // Chờ đợi chương trình bắt đầu
        waitForStart();

        // Vòng lặp chính
        while (opModeIsActive()) {
            // Đọc giá trị từ gamepad
            if (gamepad1.dpad_up) {
                // Tăng dần vị trí servo (quay lên)
                servoPosition += 0.01; // Điều chỉnh tốc độ bằng cách thay đổi giá trị này
                if (servoPosition > 1.0) {
                    servoPosition = 1.0; // Giới hạn vị trí tối đa
                }
            } else if (gamepad1.dpad_down) {
                // Giảm dần vị trí servo (quay xuống)
                servoPosition -= 0.01; // Điều chỉnh tốc độ bằng cách thay đổi giá trị này
                if (servoPosition < 0.0) {
                    servoPosition = 0.0; // Giới hạn vị trí tối thiểu
                }
            }
            if (gamepad1.dpad_up) {
                // Tăng dần vị trí servo (quay lên)
                servoPosition1 += 0.01; // Điều chỉnh tốc độ bằng cách thay đổi giá trị này
                if (servoPosition1 > 0.5) {
                    servoPosition1 = 0.5; // Giới hạn vị trí tối đa
                }
            } else if (gamepad1.dpad_down) {
                // Giảm dần vị trí servo (quay xuống)
                servoPosition1 -= 0.01; // Điều chỉnh tốc độ bằng cách thay đổi giá trị này
                if (servoPosition1 < 0.0) {
                    servoPosition1 = 0.0; // Giới hạn vị trí tối thiểu
                }
            }

            // Cập nhật vị trí servo
            Servo1.setPosition(servoPosition);
            Servo2.setPosition(1-servoPosition);
            Servo3.setPosition(servoPosition1);

            // Hiển thị vị trí hiện tại của servo lên telemetry
            telemetry.addData("Servo Position", servoPosition);
            telemetry.update();

        }
    }
}