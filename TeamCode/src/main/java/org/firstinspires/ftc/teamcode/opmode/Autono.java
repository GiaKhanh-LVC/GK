package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "Trajectory Test", group = "Test")
public class Autono extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Khởi tạo vị trí ban đầu của robot
        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);

        // Tạo một quỹ đạo đơn giản: Di chuyển về phía trước 30 inch
        Trajectory forwardTrajectory = drive.trajectoryBuilder(startPose)
                .forward(30)
                .build();

        // Thực hiện di chuyển về phía trước trước khi xoay
        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(forwardTrajectory);
        sleep(500);

        // Xoay 90 độ (Không tạo trajectory)
        drive.turn(Math.toRadians(90));
        sleep(500);

        // Cập nhật lại vị trí sau khi xoay
        Pose2d afterTurnPose = new Pose2d(forwardTrajectory.end().getX(),
                forwardTrajectory.end().getY(),
                Math.toRadians(90));

        // Tạo quỹ đạo đi theo đường cong sau khi xoay
        TrajectorySequence curveTrajectory = drive.trajectorySequenceBuilder(afterTurnPose)
                .splineToSplineHeading(new Pose2d(40, 20, Math.toRadians(0)), Math.toRadians(0))
                .build();

        drive.followTrajectorySequence(curveTrajectory);
        sleep(500);
    }
}
