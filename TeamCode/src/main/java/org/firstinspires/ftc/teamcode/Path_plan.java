package org.firstinspires.ftc.teamcode;


import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.arcrobotics.ftclib.trajectory.Trajectory;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Autonomous
public class Path_plan extends CommandOpMode {

    public MecanumDriveSubsystem drive_Subsystem;
    public Trajectory trajectory;

    @Override
    public void initialize() {
        // Initialize the drive subsystem
        drive_Subsystem = new MecanumDriveSubsystem(hardwareMap, telemetry);

        Pose2d sideStart = new Pose2d(0, 0, new Rotation2d(0));
        List<Translation2d> interiorWaypoints = Arrays.asList(
                new Translation2d(1, 0),
                new Translation2d(2, 1)
        );
        Pose2d crossScale = new Pose2d(3, 0, new Rotation2d(0));
        TrajectoryConfig config = new TrajectoryConfig(1.0, 1.0);
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                sideStart,
                interiorWaypoints,
                crossScale,
                config

        );

        schedule(new FollowTrajectoryCommand(drive_Subsystem, trajectory));
    }
    public void Path_1(){

        Pose2d sideStart_1 = new Pose2d(0, 0, new Rotation2d(0));
        List<Translation2d> interiorWaypoints_1 = Arrays.asList(
                new Translation2d(1, 0),
                new Translation2d(2, 1)
        );

        Pose2d crossScale_1 = new Pose2d(3, 0, new Rotation2d(0));
        TrajectoryConfig config_1 = new TrajectoryConfig(1.0, 1.0);
        Trajectory trajectory_1 = TrajectoryGenerator.generateTrajectory(
                sideStart_1,
                interiorWaypoints_1,
                crossScale_1,
                config_1
        );
    }
}
