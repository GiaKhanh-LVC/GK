package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Autonomous(name = "Autonomous", group = "Autonomous")
public class AutonomousMode extends LinearOpMode {

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    public MecanumDriveSubsystem drive;
    DistanceSensor ultrasonicSensor;

    static final double FEET_PER_METER = 3.28084;
    static final double GRABBED_DISTANCE_THRESHOLD = 5.0; // Ngưỡng khoảng cách để xác định vật đã gắp

    // PID Controller
    private PIDFController pidController;
    private static final double KP = 0.05; // Proportional
    private static final double KI = 0.0;  // Integral
    private static final double KD = 0.0;  // Derivative
    private static final double KF = 0.0;  // Feedforward

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    double tagsize = 0.166;

    @Override
    public void runOpMode() {
        // Initialize hardware
        ultrasonicSensor = hardwareMap.get(DistanceSensor.class, "ultrasonic_sensor");
        drive = new MecanumDriveSubsystem(hardwareMap, telemetry);

        // Initialize PID Controller
        pidController = new PIDFController(KP, KI, KD, KF);
        pidController.setSetPoint(GRABBED_DISTANCE_THRESHOLD); // Target distance
        pidController.setTolerance(0.5); // Allowable tolerance

        // Initialize Camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        camera.setPipeline(aprilTagDetectionPipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("Camera failed to open!");
                telemetry.update();
            }
        });

        telemetry.setMsTransmissionInterval(50);

        // Wait for start
        while (!isStarted() && !isStopRequested()) {
            detectAprilTags();
            sleep(20);
        }

        waitForStart();

        // Autonomous execution
        while (opModeIsActive()) {
            // Detect AprilTags
            ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getLatestDetections();

            if (!detections.isEmpty()) {
                AprilTagDetection tag = detections.get(0); // Sử dụng thẻ đầu tiên
                executeTrajectory(tag);

                // Sau khi đến gần vật, kiểm tra cảm biến sóng âm với PID
                if (checkObjectGrabbedWithPID()) {
                    telemetry.addLine("Object grabbed!");
                    telemetry.update();
                    performPostGrabActions();
                    break;
                } else {
                    telemetry.addLine("Object not grabbed, adjusting...");
                    telemetry.update();
                }
            } else {
                telemetry.addLine("No AprilTags detected, stopping.");
                telemetry.update();
                break;
            }
        }
    }

    private void detectAprilTags() {
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
    }

    private void executeTrajectory(AprilTagDetection tag) {
        Pose2d startPose = new Pose2d(tag.pose.x * FEET_PER_METER, tag.pose.y * FEET_PER_METER, new Rotation2d());
        List<Translation2d> waypoints = Arrays.asList(
                new Translation2d(1, 0),
                new Translation2d(2, 1)
        );
        Pose2d endPose = new Pose2d(3, 0, new Rotation2d(0));
        TrajectoryConfig config = new TrajectoryConfig(1.0, 1.0);
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(startPose, waypoints, endPose, config);

        drive.followTrajectory(trajectory);
    }

    private boolean checkObjectGrabbedWithPID() {
        double currentDistance = ultrasonicSensor.getDistance(DistanceUnit.CM);

        // PID calculation
        double output = pidController.calculate(currentDistance);
        output = Math.max(-1.0, Math.min(1.0, output)); // Clamp output

        // Adjust robot motors based on PID output
        drive.setMotorPowers(output, output, output, output);

        telemetry.addData("Ultrasonic Distance (cm)", currentDistance);
        telemetry.addData("PID Output", output);
        telemetry.update();

        return pidController.atSetPoint();
    }

    private void performPostGrabActions() {
        telemetry.addLine("Performing actions after grabbing object...");
        telemetry.update();
        // Thêm logic sau khi gắp được vật (ví dụ: quay về điểm xuất phát hoặc tiếp tục nhiệm vụ khác)
    }

    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
    }
}
