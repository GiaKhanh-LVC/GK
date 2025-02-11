package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


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

@Autonomous(name = "AutonomousMode_Blue", group = "Autonomous")
public class AutonomousMode extends LinearOpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    double tagsize = 0.166;

    @Override
    public void runOpMode() {
        AprilTagDetection detection=new AprilTagDetection();
        Orientation rot = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy,telemetry);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

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
                    Trajectory trajectory_1 = drive.trajectoryBuilder(startPose)
                            .lineToLinearHeading(new Pose2d(-20, 0, Math.toRadians(0))) // Di chuyển đến vị trí của xà
                            .build();

                    // Di chuyển robot theo quỹ đạo đã tạo
                    drive.followTrajectory(trajectory_1);
                    while (opModeIsActive()) {
                        // Cập nhật lại vị trí sau tra_1
                        Pose2d after_tra_1 = new Pose2d(trajectory_1.end().getX(),
                                trajectory_1.end().getY(),
                                Math.toRadians(90));

                        // Tạo quỹ đạo đi theo đường xiên
                        TrajectorySequence trajectory_2 = drive.trajectorySequenceBuilder(after_tra_1)
                                .strafeTo(new Vector2d(-72, -36))
                                .build();

                        // Di chuyển robot tới nơi lấy implement
                        drive.followTrajectorySequence(trajectory_2);


                        //CODE XU LY GAP

                        // Cập nhật lại vị trí sau tra_2
                        Pose2d after_tra_2 = new Pose2d(trajectory_2.end().getX(),
                                trajectory_2.end().getY(),
                                Math.toRadians(90));
                        // Tạo quỹ đạo đi theo xiên
                        TrajectorySequence trajectory_3 = drive.trajectorySequenceBuilder(after_tra_2)
                                .strafeTo(new Vector2d(-20, 0))
                                .build();
                        // Di chuyển robot tới nơi xà
                        drive.followTrajectorySequence(trajectory_3);


                        // CODE XU LY THA:



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
