package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class BlueDuckAuto  {


    /*
     * This is an example of a more complex path to really test the tuning.
     */
    @Autonomous(group = "drive")
    public class BlueDuckAuto extends LinearOpMode {
        @Override
        public void runOpMode() throws InterruptedException {
            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

            Pose2d startPos = new Pose2d(-48, -48, Math.toRadians(0));
            drive.setPoseEstimate(startPos);
            telemetry.update();

            waitForStart();

            if (isStopRequested()) return;

            Trajectory traj = drive.trajectoryBuilder(new Pose2d())
                    .splineTo(new Vector2d(0, 24), Math.toRadians(90))
                    .build();

            drive.followTrajectory(traj);

            sleep(2000);

            drive.followTrajectory(
                    drive.trajectoryBuilder(traj.end(), true)
                            .splineTo(new Vector2d(-24, -12), Math.toRadians(180))
                            .build()
            );
        }
    }


}
