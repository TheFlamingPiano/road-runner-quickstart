
    package org.firstinspires.ftc.teamcode.snappy;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.CyrusIntakeArmHardware;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


    /*
     * This is an example of a more complex path to really test the tuning.
     */
    @Autonomous(group = "drive")
    public class BlueParkMultiBlock extends LinearOpMode {

        final double EXTENSION_READY_DISTANCE = 0.0;
        final double EXTENSION_READY_HEIGHT = 55.0;
        CyrusIntakeArmHardware ik;

        @Override
        public void runOpMode() throws InterruptedException {
            SnappyHardware snappy = new SnappyHardware(hardwareMap,true, SnappyHardware.TeamColor.BLUE);
            CameraSnap cam = new CameraSnap();

            ik = new CyrusIntakeArmHardware(snappy.ARM1_LENGTH, snappy.ARM2_LENGTH);


            Pose2d startPos = new Pose2d(10, 60, Math.toRadians(180));
            snappy.setPoseEstimate(startPos);
            telemetry.update();
            double angles[] = ik.getAngles(EXTENSION_READY_DISTANCE, EXTENSION_READY_HEIGHT);

            snappy.BaseArm.setTargetPosition((int) ((angles[0] - snappy.INITIAL_ARM1_ANGLE) * snappy.ENCODER_TICKS_PER_DEGREE_ARM1));
            //arm.IntakeArm.setTargetPosition((int) ((angles[1] - arm.INITIAL_ARM2_ANGLE + (angles[0] - arm.INITIAL_ARM1_ANGLE) / arm.GEAR_RATIO_ARM2_STAGE) * arm.ENCODER_TICKS_PER_DEGREE_ARM2));
            snappy.IntakeArm.setTargetPosition((int) ((angles[1] - snappy.INITIAL_ARM2_ANGLE) * snappy.ENCODER_TICKS_PER_DEGREE_ARM2));
snappy.ClawServo.setPosition(0.1);
cam.runOpMode(this,true, SnappyHardware.TeamColor.BLUE);
//int position = cam.getPosition();
//while (position != 4) {
//     position = cam.getPosition();
//    telemetry.addData("position", position);
//    telemetry.addData("LeftGreen",cam.greenleft-cam.redleft-cam.blueleft);
//    telemetry.addData("MiddleGreen",cam.greenmiddle-cam.redmiddle-cam.bluemiddle);
//    telemetry.addData("RightGreen",cam.greenright-cam.redright-cam.blueright);
////
//    telemetry.update();
//}


            waitForStart();
            snappy.moveEncoderWheelDown();

            //TAKE THIS OUT LATER?!
            int position = cam.getPosition(); //PUT THIS BACK IN LATER!!!!
            telemetry.addData("LeftGreen",cam.greenleft-cam.redleft-cam.blueleft);
            telemetry.addData("MiddleGreen",cam.greenmiddle-cam.redmiddle-cam.bluemiddle);
            telemetry.addData("RightGreen",cam.greenright-cam.redright-cam.blueright);
            telemetry.addData("Position", position);
            telemetry.addData("Height", snappy.height);
            telemetry.addData("Distance", snappy.distance);
            telemetry.addData("Rotation", snappy.rotation);
            telemetry.update();
            snappy.BaseArm.setPower(1);
            snappy.IntakeArm.setPower(1);
            snappy.RotationMotor.setPower(1);

            snappy.setArmAnglesToHome(this);

            if (isStopRequested()) return;


//            TrajectorySequence trajectory2 = snappy.trajectorySequenceBuilder(trajectory1.end())
//                    .forward(29)
////                    .turn(Math.toRadians(90))
////                    .strafeRight(5)
////                    .back(35)
//                    .build();


            snappy.deliverXblocks(this,-119,position, 0);
            //snappy.setArmAnglesToHome(this);
            for (int i = 0; i < 3; i++) {
                snappy.deliverExtraBlock(SnappyHardware.TeamColor.BLUE, i, this);
            }
                TrajectorySequence trajectory1 = snappy.trajectorySequenceBuilder(startPos)
                        .back(29)
//                    .turn(Math.toRadians(90))
//                    .strafeRight(5)
//                    .back(35)
                        .build();

                snappy.followTrajectorySequence(trajectory1);
//if (i > 0) {
//    startPos = snappy.getPoseEstimate();
//}
//                TrajectorySequence trajectory1 = snappy.trajectorySequenceBuilder(startPos)
//                        .back(29 + (2*i))
////                    .turn(Math.toRadians(90))
////                    .strafeRight(5)
////                    .back(35)
//                        .build();
//                TrajectorySequence trajectory2 = snappy.trajectorySequenceBuilder(trajectory1.end())
//                        .forward(29 + (2*i))
////                    .turn(Math.toRadians(90))
////                    .strafeRight(5)
////                    .back(35)
//                        .build();
//
//                snappy.PrepickUpBlock(this);
//                snappy.followTrajectorySequence(trajectory1);
//                snappy.ClawServo.setPosition(0);
//                snappy.IntakeServo.setPosition(0.5);
//                double startTime = System.nanoTime() * 1e-9;
//                double waitTime = 1.0;
//                while ((opModeIsActive())
//                    && startTime + waitTime > System.nanoTime() * 1e-9
//                        && snappy.sensorRange.getDistance(DistanceUnit.MM) > 30.0);
//                snappy.followTrajectorySequence(trajectory2);
//                snappy.postPickUpBlock(this);
//                position = 3;
//                snappy.deliverXblocks(this, -119, position, 0);
//                snappy.StepBreakMovement(this, -10, 22, -22, 1, .5);
//            }



//            telemetry.addData("Height", snappy.height);
//            telemetry.addData("Distance", snappy.distance);
//            telemetry.addData("Rotation", snappy.rotation);
//            telemetry.update();
            //drive foward
         //   pickUpBlock(snappy);

//            telemetry.addData("Height", snappy.height);
//            telemetry.addData("Distance", snappy.distance);
//            telemetry.addData("Rotation", snappy.rotation);
//            telemetry.update();
            //drive back
          //  snappy.deliverXblocks(this,-123,position);

           // pickUpBlock(snappy);

          //  snappy.deliverXblocks(this,-123,position);

//            telemetry.addData("Height", snappy.height);
//            telemetry.addData("Distance", snappy.distance);
//            telemetry.addData("Rotation", snappy.rotation);
//            telemetry.update();


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

//        private void pickUpBlock(SnappyHardware snappy) {
//            snappy.StepBreakMovement(this, 0, 40, 0, 1, (long)1);
//            snappy.StepBreakMovement(this, 0, 80, -10, 1,(long)1);
//            snappy.IntakeServo.setPosition(1);
//            snappy.ClawServo.setPosition(0.2);
//            snappy.wait(this, 1);
//            snappy.IntakeServo.setPosition(0.5);
//            snappy.ClawServo.setPosition(0);
//            snappy.StepBreakMovement(this, 0, 50, 40, 1, (long)1);
//            snappy.StepBreakMovement(this, snappy.INITIAL_ROTATION_ANGLE,91,-10,1,1);
//        }
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






