package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import java.util.ArrayList;
import org.firstinspires.ftc.teamcode.config.action_active;
@Autonomous(name = "AutonomousMode_Red", group = "Autonomous")
public class AutonoModeRed extends LinearOpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    double tagsize = 0.166;
    public Servo Servo1,Servo2,Claw;
    public DcMotorEx motor_encode;
    public action_active action_active=new action_active();

    @Override
    public void runOpMode() {
        motor_encode=hardwareMap.get(DcMotorEx.class,"motor_encode");
        motor_encode.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        AprilTagDetection detection=new AprilTagDetection();
        Orientation rot = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy,telemetry);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

//        // Set PID coefficients for better control
//        PIDFCoefficients pidCoefficients = new PIDFCoefficients(2.0, 0.1, 0.2, 0.0);
//        drive.setPIDCoefficients(SampleMecanumDrive.MotorType.LEFT_FRONT, pidCoefficients);
//        drive.setPIDCoefficients(SampleMecanumDrive.MotorType.RIGHT_FRONT, pidCoefficients);
//        drive.setPIDCoefficients(SampleMecanumDrive.MotorType.LEFT_REAR, pidCoefficients);
//        drive.setPIDCoefficients(SampleMecanumDrive.MotorType.RIGHT_REAR, pidCoefficients);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        telemetry.setMsTransmissionInterval(50);

        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (!currentDetections.isEmpty()) {
                for (AprilTagDetection tag : currentDetections) {
                    telemetry.addLine("Tag detected:");
                    tagToTelemetry(tag);
                }
            } else {
                telemetry.addLine("No AprilTags detected");
            }
            telemetry.update();
            sleep(20);
        }

        while (opModeIsActive()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
            if (!currentDetections.isEmpty()) {
                for (AprilTagDetection tag : currentDetections) {
                    telemetry.addLine("Tag detected:");
                    tagToTelemetry(tag);
                    // Xử lý quỹ đạo và di chuyển robot
                    // Nếu tag được phát hiện, lấy tọa độ và tạo trajectory
                    double tagX = tag.pose.x * FEET_PER_METER; // Chuyển đổi từ mét sang feet
                    double tagY = tag.pose.y * FEET_PER_METER;
                    double tagYaw = rot.firstAngle; // Yaw rotation của tag

                    Pose2d startPose = new Pose2d(tagX, tagY, Math.toRadians(tagYaw));
                    drive.setPoseEstimate(startPose);

                    // Tạo trajectory đến vị trí của tag
                    TrajectorySequence trajectory_1 = drive.trajectorySequenceBuilder(startPose)
                            .lineToLinearHeading(new Pose2d(20, 0, Math.toRadians(180))) // Di chuyển đến vị trí của xà
                            .build();

                    // Di chuyển robot theo quỹ đạo đã tạo
                    drive.followTrajectorySequence(trajectory_1);
                    action_active.champer(Servo1,Servo2,motor_encode);
                    action_active.reset(Servo1,Servo2,motor_encode);
                    Claw.setPosition(1);//MỞ
                    Claw.setPosition(0);//ĐÓNG ĐIỀU CHỈNH THÔNG SỐ THỰC TẾ
                    Pose2d after_tra_1 = new Pose2d(trajectory_1.end().getX(),
                            trajectory_1.end().getY(),
                            Math.toRadians(90));
                    TrajectorySequence trajectory = drive.trajectorySequenceBuilder(after_tra_1)
//                            .back(24)
//                            .strafeRight(12)
//                            .forward(36)
                            .lineToSplineHeading(new Pose2d(60, -24, Math.toRadians(90)))
                            .strafeLeft(60)
                            .strafeRight(60)
                            .back(12)
                            .strafeLeft(60)
                            .strafeRight(60)
                            .back(12)
                            .lineToSplineHeading(new Pose2d(72,36,Math.toRadians(270)))// Di chuyển đến vị trí lấy implementr

                            .build();
                    drive.followTrajectorySequence(trajectory);
                    while (opModeIsActive()) {
                        // Cập nhật lại vị trí sau tra_1
                        Pose2d after_tra = new Pose2d(trajectory.end().getX(),
                                trajectory.end().getY(),
                                Math.toRadians(270));

                        // Tạo quỹ đạo đi theo đường xiên
                        TrajectorySequence trajectory_2 = drive.trajectorySequenceBuilder(after_tra_1)
                                .lineToSplineHeading(new Pose2d(72,-36,Math.toRadians(90)))
                                .build();

                        // Di chuyển robot tới nơi lấy implement
                        drive.followTrajectorySequence(trajectory_2);
                        action_active.fence(Servo1, Servo2,motor_encode);
                        action_active.reset(Servo1,Servo2,motor_encode);
                        Claw.setPosition(1);//MỞ
                        Claw.setPosition(0);//ĐÓNG
                        //ĐẨY CÁC PHẦN TỬ VÀO VỊ TRÍ LẤY IMPLEMENT
                        Pose2d after_tra_2=new Pose2d(trajectory_2.end().getX(),trajectory_2.end().getY());





                        // Tạo quỹ đạo đi theo xiên
                        TrajectorySequence trajectory_3 = drive.trajectorySequenceBuilder(after_tra_2)
                                .lineToSplineHeading(new Pose2d(-20,0,Math.toRadians(0)))
                                .build();
                        // Di chuyển robot tới nơi xà
                        drive.followTrajectorySequence(trajectory_3);


                        // CODE XU LY THA:
                        action_active.champer(Servo1,Servo2,motor_encode);
                        action_active.reset(Servo1,Servo2,motor_encode);
                        Claw.setPosition(1);//ĐIỀN DỮ LIỆU THỰC TẾ
                        Claw.setPosition(0);//ĐIỀN DỮ LIỆU THỰC TẾ


                    }

                }
            } else {
                telemetry.addLine("No AprilTags detected");
            }
            telemetry.update();
            sleep(20);
        }
    }

    void tagToTelemetry(AprilTagDetection detection) {
        Orientation rot = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);


        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", rot.firstAngle));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", rot.secondAngle));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", rot.thirdAngle));
    }
}
