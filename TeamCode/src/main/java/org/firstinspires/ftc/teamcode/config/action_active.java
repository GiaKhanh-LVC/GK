package org.firstinspires.ftc.teamcode.config;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class action_active {
    public void basket(Servo Servo1, Servo Servo2, DcMotorEx Motor_encode) {
        int ticks=0;//ĐIỀN SỐ
        double servo_position=0;//ĐIỀN SỐ
        // Đặt vị trí cho servo (ví dụ: servo_position là vị trí bạn muốn)
        Servo1.setPosition(servo_position);
        Servo2.setPosition(servo_position);

        // Cấu hình động cơ
        Motor_encode.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);  // Đặt lại encoder
        Motor_encode.setTargetPosition((int)ticks);  // Đặt vị trí đích cho động cơ
        Motor_encode.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);  // Chế độ chạy tới vị trí

//        // Di chuyển đồng thời cả servo và động cơ
//        while (Motor_encode.isBusy()) {
//            // Động cơ di chuyển tới vị trí mục tiêu, servo đã di chuyển tới vị trí ban đầu
//            telemetry.addData("Motor Position", Motor_encode.getCurrentPosition());
//            telemetry.addData("Servo1 Position", Servo1.getPosition());
//            telemetry.addData("Servo2 Position", Servo2.getPosition());
//            telemetry.update();
//        }
//
//        // Sau khi động cơ đạt đến vị trí mục tiêu, quay lại vị trí ban đầu
//        Servo1.setPosition(0.5); // Quay lại vị trí ban đầu của servo
//        Servo2.setPosition(0.5); // Quay lại vị trí ban đầu của servo
//        Motor_encode.setTargetPosition(-(int)ticks);  // Di chuyển động cơ về vị trí ban đầu
//
//        // Đợi động cơ quay lại vị trí ban đầu
//        while (Motor_encode.isBusy()) {
//            telemetry.addData("Motor Position", Motor_encode.getCurrentPosition());
//            telemetry.update();
//        }
//
//        // Dừng động cơ khi hoàn thành
//        Motor_encode.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);  //


    }

    public void champer(Servo Servo1, Servo Servo2, DcMotorEx Motor_encode) {
        int ticks=0;//ĐIỀN SỐ
        double servo_position=0;//ĐIỀN SỐ
        // Đặt vị trí cho servo (ví dụ: servo_position là vị trí bạn muốn)
        Servo1.setPosition(servo_position);
        Servo2.setPosition(servo_position);

        // Cấu hình động cơ
        Motor_encode.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);  // Đặt lại encoder
        Motor_encode.setTargetPosition((int)ticks);  // Đặt vị trí đích cho động cơ
        Motor_encode.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);  // Chế độ chạy tới vị trí

//        // Di chuyển đồng thời cả servo và động cơ
//        while (Motor_encode.isBusy()) {
//            // Động cơ di chuyển tới vị trí mục tiêu, servo đã di chuyển tới vị trí ban đầu
//            telemetry.addData("Motor Position", Motor_encode.getCurrentPosition());
//            telemetry.addData("Servo1 Position", Servo1.getPosition());
//            telemetry.addData("Servo2 Position", Servo2.getPosition());
//            telemetry.update();
//        }
//
//        // Sau khi động cơ đạt đến vị trí mục tiêu, quay lại vị trí ban đầu
//        Servo1.setPosition(0.5); // Quay lại vị trí ban đầu của servo
//        Servo2.setPosition(0.5); // Quay lại vị trí ban đầu của servo
//        Motor_encode.setTargetPosition(-(int)ticks);  // Di chuyển động cơ về vị trí ban đầu
//
//        // Đợi động cơ quay lại vị trí ban đầu
//        while (Motor_encode.isBusy()) {
//            telemetry.addData("Motor Position", Motor_encode.getCurrentPosition());
//            telemetry.update();
//        }
//
//        // Dừng động cơ khi hoàn thành
//        Motor_encode.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);  //


    }
    public void fence(Servo Servo1, Servo Servo2, DcMotorEx Motor_encode) {
        int ticks=0;//ĐIỀN SỐ
        double servo_position=0;//ĐIỀN SỐ
        Motor_encode.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        // Đặt vị trí cho servo (ví dụ: servo_position là vị trí bạn muốn)
        Servo1.setPosition(servo_position);
        Servo2.setPosition(servo_position);

        // Cấu hình động cơ
          // Đặt lại encoder
        Motor_encode.setTargetPosition((int)ticks);  // Đặt vị trí đích cho động cơ
        Motor_encode.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);  // Chế độ chạy tới vị trí

//        // Di chuyển đồng thời cả servo và động cơ
//        while (Motor_encode.isBusy()) {
//            // Động cơ di chuyển tới vị trí mục tiêu, servo đã di chuyển tới vị trí ban đầu
//            telemetry.addData("Motor Position", Motor_encode.getCurrentPosition());
//            telemetry.addData("Servo1 Position", Servo1.getPosition());
//            telemetry.addData("Servo2 Position", Servo2.getPosition());
//            telemetry.update();
//        }
//
//        // Sau khi động cơ đạt đến vị trí mục tiêu, quay lại vị trí ban đầu
//        Servo1.setPosition(0.5); // Quay lại vị trí ban đầu của servo
//        Servo2.setPosition(0.5); // Quay lại vị trí ban đầu của servo
//        Motor_encode.setTargetPosition(-(int)ticks);  // Di chuyển động cơ về vị trí ban đầu
//
//        // Đợi động cơ quay lại vị trí ban đầu
//        while (Motor_encode.isBusy()) {
//            telemetry.addData("Motor Position", Motor_encode.getCurrentPosition());
//            telemetry.update();
//        }
//
//        Motor_encode.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);  //


    }
    public void reset(Servo servo1,Servo servo2,DcMotorEx motor){
        servo1.setPosition(0);
        servo2.setPosition(0);
        motor.setTargetPosition(0);
        motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }
}
