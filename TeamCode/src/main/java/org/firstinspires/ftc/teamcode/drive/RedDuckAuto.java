package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;


    /*
     * This is an example of a more complex path to really test the tuning.
     */
    @Autonomous(group = "drive")
    public class RedDuckAuto extends LinearOpMode {
        @Override
        public void runOpMode() throws InterruptedException {
            CyrusOfficialHardware drive = new CyrusOfficialHardware(hardwareMap);
            CyrusIntakeArmHardware arm = new CyrusIntakeArmHardware(hardwareMap);
            CyrusCarouselHardware duck = new CyrusCarouselHardware(hardwareMap);

            Pose2d startPos = new Pose2d(-35, -60, Math.toRadians(270));
            drive.setPoseEstimate(startPos);
            telemetry.update();

            waitForStart();

            if (isStopRequested()) return;

            TrajectorySequence trajectory1 = drive.trajectorySequenceBuilder(startPos)
                    .back(8)
                    .strafeRight(33)
                    .forward(4)
                    .build();

            TrajectorySequence trajectory2 = drive.trajectorySequenceBuilder(trajectory1.end())
                    .waitSeconds(4)
                    .back(25)
                    .build();

            drive.followTrajectorySequence(trajectory1);
            duck.RedSpin();
            drive.followTrajectorySequence(trajectory2);
            duck.Stop();

//        Trajectory traj = drive.trajectoryBuilder(new Pose2d())
//                .splineTo(new Vector2d(0, 24), Math.toRadians(90))
//                .build();
//
//        drive.followTrajectory(traj);
//
//        sleep(2000);
//
//        drive.followTrajectory(
//                drive.trajectoryBuilder(traj.end(), true)
//                        .splineTo(new Vector2d(-24, -12), Math.toRadians(180))
//                        .build()
            //);
        }
    }





    /*
     * This is an example of a more complex path to really test the tuning.
     */
    //     @Override
    //    public void runOpMode() throws InterruptedException {
    //       SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    //      Pose2d startPos = new Pose2d(-48, -48, Math.toRadians(0));
    //       drive.setPoseEstimate(startPos);
    //       telemetry.update();

    //    waitForStart();

    //      if (isStopRequested()) return;

    //      Trajectory traj = drive.trajectoryBuilder(new Pose2d())
    //              .splineTo(new Vector2d(0, 24), Math.toRadians(90))
    //              .build();

    //      drive.followTrajectory(traj);

    //      sleep(2000);

    //    drive.followTrajectory(
    //             drive.trajectoryBuilder(traj.end(), true)
    //                     .splineTo(new Vector2d(-24, -12), Math.toRadians(180))
//build()
    //     );
    //   }





