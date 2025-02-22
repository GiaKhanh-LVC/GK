package org.firstinspires.ftc.teamcode.config;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MotorControlModule {
    public DcMotorEx leftFront,rightFront,leftRear,rightRear,roll_motor;

    public MotorControlModule() {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        roll_motor = hardwareMap.get(DcMotorEx.class, "roll_motor");
        roll_motor.setDirection(DcMotor.Direction.FORWARD);
        roll_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        roll_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        roll_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void m_up(){
        int ticks=roll_motor.getCurrentPosition();
        ticks+=448;
        roll_motor.setTargetPosition((int)ticks);
        roll_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    public void m_down(){
        int ticks=roll_motor.getCurrentPosition();
        ticks-=448;
        roll_motor.setTargetPosition((int)ticks);
        roll_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);}

}
