package org.firstinspires.ftc.teamcode.snappy;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.CyrusCarouselHardware;
import org.firstinspires.ftc.teamcode.drive.CyrusIntakeArmHardware;
import org.firstinspires.ftc.teamcode.drive.CyrusOfficialHardware;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class BlueDuckSnappy extends LinearOpMode {
    final double EXTENSION_READY_DISTANCE = 0.0;
    final double EXTENSION_READY_HEIGHT = 55.0;

    CyrusIntakeArmHardware ik;
    @Override

    public void runOpMode() throws InterruptedException {

        SnappyHardware snappy = new SnappyHardware(hardwareMap,true);

        ik = new CyrusIntakeArmHardware(snappy.ARM1_LENGTH, snappy.ARM2_LENGTH);

        Pose2d startPos = new Pose2d(-35, 60, Math.toRadians(90));
        snappy.setPoseEstimate(startPos);
        telemetry.update();

        double angles[] = ik.getAngles(EXTENSION_READY_DISTANCE, EXTENSION_READY_HEIGHT);

        snappy.BaseArm.setTargetPosition((int) ((angles[0] - snappy.INITIAL_ARM1_ANGLE) * snappy.ENCODER_TICKS_PER_DEGREE_ARM1));
        //arm.IntakeArm.setTargetPosition((int) ((angles[1] - arm.INITIAL_ARM2_ANGLE + (angles[0] - arm.INITIAL_ARM1_ANGLE) / arm.GEAR_RATIO_ARM2_STAGE) * arm.ENCODER_TICKS_PER_DEGREE_ARM2));
        snappy.IntakeArm.setTargetPosition((int) ((angles[1] - snappy.INITIAL_ARM2_ANGLE) * snappy.ENCODER_TICKS_PER_DEGREE_ARM2));
        snappy.DumpDoor.setPosition(0.3);
        waitForStart();
        snappy.BaseArm.setPower(0.5);
        snappy.IntakeArm.setPower(0.5);
        if (isStopRequested()) return;

        TrajectorySequence trajectory1 = snappy.trajectorySequenceBuilder(startPos)
                .back(4)
                .strafeLeft(28)
                .turn(Math.toRadians(45))
                .build();

        TrajectorySequence trajectory2 = snappy.trajectorySequenceBuilder(trajectory1.end())
                .forward(5)
                .build();
        TrajectorySequence trajectory3 = snappy.trajectorySequenceBuilder(trajectory2.end())
                .waitSeconds(4)
                .turn(Math.toRadians(-45))
                .back(26)
                .build();





        snappy.deliverXblocks(this,23,3);
        snappy.followTrajectorySequence(trajectory1);
        snappy.followTrajectorySequence(trajectory2);
        snappy.BlueSpin();
        snappy.followTrajectorySequence(trajectory3);
        snappy.StopCarousel();
        snappy.DumpDoor.setPosition(.5);

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



