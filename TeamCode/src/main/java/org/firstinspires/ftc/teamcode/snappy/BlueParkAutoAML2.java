
    package org.firstinspires.ftc.teamcode.snappy;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


    /*
     * This is an example of a more complex path to really test the tuning.
     */
    @Disabled
    @Autonomous(group = "drive")
    public class BlueParkAutoAML2 extends LinearOpMode {

        final double EXTENSION_READY_DISTANCE = 0.0;
        final double EXTENSION_READY_HEIGHT = 55.0;
        InverseKinematicsSnap ik;

        @Override
        public void runOpMode() throws InterruptedException {
            SnappyHardware snappy = new SnappyHardware(hardwareMap, true, SnappyHardware.TeamColor.BLUE, SnappyHardware.EncoderPosition.DOWN);

            ik = new InverseKinematicsSnap(snappy.ARM1_LENGTH, snappy.ARM2_LENGTH);


            Pose2d startPos = new Pose2d(10, 60, Math.toRadians(90));
            snappy.setPoseEstimate(startPos);
            telemetry.update();
            double angles[] = ik.getAngles(snappy.INITIAL_DISTANCE, snappy.INITIAL_HEIGHT);

            telemetry.addData("arm 1 angle", angles[0]);
            telemetry.addData("arm 2 angle", angles[1]);
            telemetry.update();
            snappy.BaseArm.setTargetPosition((int) ((angles[0] - snappy.INITIAL_ARM1_ANGLE) * snappy.ENCODER_TICKS_PER_DEGREE_ARM1));
            //arm.IntakeArm.setTargetPosition((int) ((angles[1] - arm.INITIAL_ARM2_ANGLE + (angles[0] - arm.INITIAL_ARM1_ANGLE) / arm.GEAR_RATIO_ARM2_STAGE) * arm.ENCODER_TICKS_PER_DEGREE_ARM2));
            snappy.IntakeArm.setTargetPosition((int) ((angles[1] - snappy.INITIAL_ARM2_ANGLE) * snappy.ENCODER_TICKS_PER_DEGREE_ARM2));


            snappy.ClawServo.setPosition(0.3);
            waitForStart();

            // top level
//            double firstHeight = 350;
//            double firstDistance = 150;
//            double firstWrist = 45;
//            double targetHeight = 425;
//            double targetDistance = 670;
//            double wristTargetAngle = -30;
//            double outtakePower = 0;
            // middle level
//            double firstHeight = 275;
//            double firstDistance = 200;
//            double firstWrist = 45;
//            double targetHeight = 275;
//            double targetDistance = 570;
//            double wristTargetAngle = 0;
//            double outtakePower = 0;
            // middle level
            double firstHeight = 85;
            double firstDistance = 200;
            double firstWrist = 45;
            double targetHeight = 85;
            double targetDistance = 550;
            double wristTargetAngle = 0;
            double outtakePower = 0;
            double targetRotationAngle = 25;

            // figure out the arm angles for the target position so we can set the wrist to the correct angle
            angles = ik.getAngles(targetDistance, targetHeight);
            double wristPosition= (0.5)+((wristTargetAngle - (angles[0]- snappy.INITIAL_ARM1_ANGLE) - (angles[1]- snappy.INITIAL_ARM2_ANGLE))/180);
            long startTime = System.nanoTime();

            snappy.ClawServo.setPosition(0.3);

            snappy.moveArmToPosition (this, -25, firstDistance, firstHeight, firstWrist);
            startTime = System.nanoTime();
            while ((startTime + 1.0 * 1e9 > System.nanoTime()) && !isStopRequested()) {
                // just wait
            }
            //
            snappy.moveArmToPosition (this, targetRotationAngle, firstDistance, firstHeight, firstWrist);
            startTime = System.nanoTime();
            while ((startTime + 1.0 * 1e9 > System.nanoTime()) && !isStopRequested()) {
                // just wait
            }
            snappy.moveArmToPosition (this, targetRotationAngle, targetDistance, targetHeight, wristPosition);
            // should be a wait method in SnappyHardware
            startTime = System.nanoTime();
            while ((startTime + 0.5 * 1e9 > System.nanoTime()) && !isStopRequested()) {
                // just wait
            }
            // should be a method in SnappyHardware
            snappy.ClawServo.setPosition(0.55);
            snappy.IntakeServo.setPosition(outtakePower);


            startTime = System.nanoTime();
            while ((startTime + 1 * 1e9 > System.nanoTime()) && !isStopRequested()) {
                // just wait
            }

            snappy.ClawServo.setPosition(0.3);
            snappy.IntakeServo.setPosition(0);
            snappy.moveArmToPosition(this, -35, snappy.SAFE_POSITION_DISTANCE, snappy.SAFE_POSITION_HEIGHT, 0.85);

            startTime = System.nanoTime();
            while ((startTime + 1 * 1e9 > System.nanoTime()) && !isStopRequested()) {
                // just wait
            }

//            snappy.BaseArm.setPower(0.5);
//            snappy.IntakeArm.setPower(0.5);
            if (isStopRequested()) return;

//            TrajectorySequence trajectory1 = snappy.trajectorySequenceBuilder(startPos)
//                    .back(4)
//                    .turn(Math.toRadians(90))
//                    .strafeRight(10)
//                    .back(30)
//                    .build();
//
//
//            snappy.followTrajectorySequence(trajectory1);







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






