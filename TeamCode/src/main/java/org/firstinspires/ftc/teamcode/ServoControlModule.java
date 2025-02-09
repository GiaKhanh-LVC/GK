package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
public class ServoControlModule {
    public Servo servo1;
    public Servo servo2;
    public Servo servo3,servo4;
/////////// SERVO 270 //////
    // Constructor that takes HardwareMap and servo name to initialize the servo
    public ServoControlModule() {
        servo2 = hardwareMap.get(Servo.class, "xoay nguoc");
        servo2.setPosition(0.17); // Set 45 degree
        servo2.setDirection(Servo.Direction.FORWARD);
        servo1 = hardwareMap.get(Servo.class, "claw");
        servo1.setPosition(0.3); // Set 90 degree
        servo1.setDirection(Servo.Direction.FORWARD);
        servo3 = hardwareMap.get(Servo.class, "xoay truc");
        servo3.setPosition(0.5); // Set 135 degree
        servo3.setDirection(Servo.Direction.FORWARD);
    }

    // Method to close the claw
    public void closeClaw( Servo servo) {
            servo.setPosition(0.3); // 90 degree
    }

    // Method to open the claw
    public void openClaw(Servo servo) {
            servo.setPosition(0.0); // 0 degree
    }
    public void down(Servo servo1){
        servo1.setPosition(0);// 0 degree

    }
    public void up(Servo servo1){
        servo1.setPosition(0.17);//45 degree

    }
    public void fold_up(Servo servo1){
        servo1.setPosition(0.84);//225 degree

    }
    public void fold_down(Servo servo1){
        servo1.setPosition(0.17);//45 degree
         }
    public void tracking(Servo servo1){

    }

}
