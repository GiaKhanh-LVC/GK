package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.trajectory.Trajectory;

public class FollowTrajectoryCommand extends CommandBase {
    private final MecanumDriveSubsystem driveSubsystem;
    public Trajectory trajectory;

    public FollowTrajectoryCommand(MecanumDriveSubsystem driveSubsystem, Trajectory trajectory) {
        this.driveSubsystem = driveSubsystem;
        this.trajectory = trajectory;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        driveSubsystem.followTrajectory(trajectory);
    }

    @Override
    public void execute() {
        driveSubsystem.update();
    }

    @Override
    public boolean isFinished() {
        return driveSubsystem.isTrajectoryFinished();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            driveSubsystem.stop();
        }
    }
}
