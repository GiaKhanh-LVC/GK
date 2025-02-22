package org.firstinspires.ftc.teamcode.demo_function;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "test_auto", group = "Autonomous")
public class Path_plan extends CommandOpMode {

    public SampleMecanumDrive drive_Subsystem;
    public TrajectorySequence trajectory;
    public Pose2d start_pose;

    @Override
    public void initialize() {
        // Initialize the drive subsystem
        drive_Subsystem = new SampleMecanumDrive(hardwareMap);
        start_pose = new Pose2d(0, 0, Math.toRadians(0));
        drive_Subsystem.setPoseEstimate(start_pose);
        TrajectorySequence trajectory = drive_Subsystem.trajectorySequenceBuilder(start_pose)
                .forward(24)
                .strafeRight(12)
                .forward(36)
                .lineToSplineHeading(new Pose2d(60, -24, Math.toRadians(90)))
                .strafeLeft(60)
                .strafeRight(60)
                .back(12)
                .strafeLeft(60)
                .strafeRight(60)
                .back(12)
                .strafeLeft(60)// Di chuyển đến vị trí lấy implementr
                .lineToSplineHeading(new Pose2d(72,-36,Math.toRadians(90)))
                .build();
        drive_Subsystem.followTrajectorySequence(trajectory);
        telemetry.addData("Status", "finish trajectory");


    }

}
