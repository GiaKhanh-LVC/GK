package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
public class HexMotorModule {


    public static class HexControlModule{

        public DcMotor motor;
        public void runOpMode(DcMotor motor) {

            motor = hardwareMap.get(DcMotor.class, "motorHex");
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setDirection(DcMotor.Direction.FORWARD);
        }
        public void SlideUp(DcMotor motor){
            motor.setPower(1);
        }
        public void SlideDown(DcMotor motor){
            motor.setPower(-1);
        }
    }

}
