package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
public class ServoControlModule {
    public Servo servo;
    double servoPosition ;
/////////// SERVO 270 //////
    // Constructor that takes HardwareMap and servo name to initialize the servo
public ServoControlModule(String name, HardwareMap hardwareMap) {
    servo = hardwareMap.get(Servo.class, name);
}

    // Method to close the claw
    public void closeClaw( Servo servo) {
    servoPosition=servo.getPosition();
    servo.setPosition(0.3); // chỉnh góc
    }

    // Method to open the claw
    public void openClaw(Servo servo) {
    servo.setPosition(0.0); // chỉnh góc
    }
    public void up(Servo servo){
        servoPosition=servo.getPosition();
        servoPosition += 0.01; // Điều chỉnh tốc độ bằng cách thay đổi giá trị này
        if (servoPosition > 1.0) {
            servoPosition = 1.0; // Giới hạn vị trí tối đa
        }

    }
    public void down(Servo servo){
        servoPosition=servo.getPosition();
        servoPosition -= 0.01; // Điều chỉnh tốc độ bằng cách thay đổi giá trị này
        if (servoPosition < 0.0) {
            servoPosition = 0.0; // Giới hạn vị trí tối thiểu
        }

    }


}
