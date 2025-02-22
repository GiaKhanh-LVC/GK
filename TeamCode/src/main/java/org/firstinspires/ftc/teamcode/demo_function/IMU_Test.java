package org.firstinspires.ftc.teamcode.demo_function;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="test_IMU")
public class IMU_Test extends LinearOpMode {
    private DcMotor leftFront, leftRear, rightFront, rightRear;
    private IMU.Parameters imuParams;
    private IMU imu;
    private  double globalAngle;

    private final double Kp=0.02;
    private final double Ki=0.0001;
    private final double Kd=0.002;

    @Override
    public void runOpMode() throws InterruptedException{
        leftFront=hardwareMap.get(DcMotor.class,"leftFront");
        leftRear=hardwareMap.get(DcMotor.class,"leftRear");
        rightFront=hardwareMap.get(DcMotor.class,"rightFront");
        rightRear=hardwareMap.get(DcMotor.class,"rightRear");

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        this.imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );

        this.imu = hardwareMap.get(IMU.class, "imu");
        this.imu.initialize(this.imuParams);

        telemetry.addData("IMU Status", "Initialized");
        telemetry.update();
        waitForStart();
        if (opModeIsActive()){
            turnToAngle(90,0.5);//xoay vowis gocs 90 dodoj vowis toocs ddoo toi da 50%
        }
    }
    private  double getAngle(){
        Orientation angles = this.imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }
    public void turnToAngle(double targetAngle, double maxSpeed){
        double error, prevError =0, integral=0, derivative;
        double power;
        long lastTime =System.currentTimeMillis();
        while (opModeIsActive()){
            error =targetAngle-getAngle();

            if (Math.abs(error)<1) break;
            long currentTime= System.currentTimeMillis();
            double deltaTime =(currentTime - lastTime)/1000.0;
            lastTime=currentTime;

            integral+= error * deltaTime;
            derivative=(error-prevError)/deltaTime;
            prevError=error;

            power=(Kp*error) +(Ki*integral)+(Kd * deltaTime);
            power=Math.max(-maxSpeed,Math.min(maxSpeed,power));

            leftFront.setPower(-power);
            leftRear.setPower(-power);
            rightRear.setPower(power);
            rightFront.setPower(power);

            telemetry.addData("Target Angle",targetAngle);
            telemetry.addData("Current Angle",getAngle());
            telemetry.addData("Power",power);
            telemetry.update();
        }
        stopMotors();
    }
    private void stopMotors(){
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
    }
}
