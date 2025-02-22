package org.firstinspires.ftc.teamcode.demo_function;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "PIDF Tuner", group = "Tuning")
@Config  // Cho phép chỉnh thông số PIDF trên FTC Dashboard
public class Tune extends LinearOpMode {

    private DcMotorEx motor;
    public static double Kp = 1.0;
    public static double Ki = 0.0;
    public static double Kd = 0.0;
    public static double Kf = 10.0;  // Hệ số Feedforward (tùy chỉnh nếu cần)

    @Override
    public void runOpMode() {
        // Kết nối FTC Dashboard để điều chỉnh thông số PIDF
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        // Lấy động cơ từ HardwareMap
        motor = hardwareMap.get(DcMotorEx.class, "motor");

        // Đặt chế độ encoder
        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Lấy thông số PIDF hiện tại của động cơ
        PIDFCoefficients currentPIDF = motor.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Current kP", currentPIDF.p);
        telemetry.addData("Current kI", currentPIDF.i);
        telemetry.addData("Current kD", currentPIDF.d);
        telemetry.addData("Current kF", currentPIDF.f);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Cập nhật thông số PIDF trong thời gian thực
            motor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(Kp, Ki, Kd, Kf));

            // Hiển thị các thông số hiện tại
            telemetry.addData("Updated kP", Kp);
            telemetry.addData("Updated kI", Ki);
            telemetry.addData("Updated kD", Kd);
            telemetry.addData("Updated kF", Kf);
            telemetry.addData("Motor Velocity", motor.getVelocity());
            telemetry.update();
        }
    }
}
