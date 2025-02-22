package org.firstinspires.ftc.teamcode.demo_function;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "test spline ", group = "Autonomous")
public class Test extends CommandOpMode {

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

                .splineTo(new Vector2d(30, 30), Math.PI / 2)
                .splineTo(new Vector2d(0, 60), Math.PI)
                .build();
        drive_Subsystem.followTrajectorySequence(trajectory);
        telemetry.addData("Status", "finish trajectory");


    }

}
