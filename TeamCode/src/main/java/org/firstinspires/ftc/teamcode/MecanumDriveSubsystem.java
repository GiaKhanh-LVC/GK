package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;

public class MecanumDriveSubsystem extends SubsystemBase {

    private final MecanumDrive drive;
    private final Motor leftFront, rightFront, leftRear, rightRear;
    private final Telemetry telemetry;
    private Trajectory currentTrajectory;
    private ElapsedTime timer;

    public MecanumDriveSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Initialize motors with updated names
        leftFront = new Motor(hardwareMap, "leftFront");
        rightFront = new Motor(hardwareMap, "rightFront");
        leftRear = new Motor(hardwareMap, "leftRear");
        rightRear = new Motor(hardwareMap, "rightRear");

        // Invert motors if necessary
        leftFront.setInverted(true);
        leftRear.setInverted(true);

        // Create the MecanumDrive object
        drive = new MecanumDrive(leftFront, rightFront, leftRear, rightRear);

        // Initialize timer for trajectory following
        timer = new ElapsedTime();
    }

    public void drive(double strafe, double forward, double turn) {
        drive.driveRobotCentric(strafe, forward, turn);
    }

    public void stop() {
        drive.stop();
    }

    public void followTrajectory(Trajectory trajectory) {
        this.currentTrajectory = trajectory;
        timer.reset();
        telemetry.addData("Trajectory", "Started");
        telemetry.update();
    }

    public boolean isTrajectoryFinished() {
        if (currentTrajectory == null) {
            return true;
        }
        // Assuming a simple time-based trajectory completion
        return timer.seconds() > currentTrajectory.getTotalTimeSeconds();
    }

    public void update() {
        if (currentTrajectory != null) {
            double elapsedTime = timer.seconds();
            if (elapsedTime <= currentTrajectory.getTotalTimeSeconds()) {
                // Get the target state from the trajectory at the current time
                Trajectory.State targetState = currentTrajectory.sample(elapsedTime);

                // Extract target velocity components
                double vx = targetState.velocityMetersPerSecond * Math.cos(targetState.poseMeters.getHeading());
                double vy = targetState.velocityMetersPerSecond * Math.sin(targetState.poseMeters.getHeading());
                double omega = targetState.curvatureRadPerMeter * targetState.velocityMetersPerSecond;

                // Command the drive with the calculated velocities
                drive.driveRobotCentric(vx, vy, omega);

                telemetry.addData("Trajectory", "Following");
                telemetry.addData("Time", elapsedTime);
                telemetry.addData("Target Pose", targetState.poseMeters);
                telemetry.update();
            } else {
                stop();
                telemetry.addData("Trajectory", "Completed");
                telemetry.update();
            }
        }
    }

    public void setMotorPowers(double fl, double fr, double bl, double br) {
        // Map the motor powers to the new motor names
        leftFront.set(fl);
        rightFront.set(fr);
        leftRear.set(bl);
        rightRear.set(br);
    }
}
