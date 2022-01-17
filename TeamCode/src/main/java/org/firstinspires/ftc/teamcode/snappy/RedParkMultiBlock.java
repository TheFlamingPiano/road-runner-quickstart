
    package org.firstinspires.ftc.teamcode.snappy;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.CyrusIntakeArmHardware;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


    /*
     * This is an example of a more complex path to really test the tuning.
     */
    @Autonomous(group = "drive")
    public class RedParkMultiBlock extends LinearOpMode {

        final double EXTENSION_READY_DISTANCE = 0.0;
        final double EXTENSION_READY_HEIGHT = 55.0;
        CyrusIntakeArmHardware ik;

        @Override
        public void runOpMode() throws InterruptedException {
            SnappyHardware snappy = new SnappyHardware(hardwareMap,true, SnappyHardware.TeamColor.RED, SnappyHardware.EncoderPosition.DOWN);
            CameraSnap cam = new CameraSnap();

            ik = new CyrusIntakeArmHardware(snappy.ARM1_LENGTH, snappy.ARM2_LENGTH);

snappy.INITIAL_ROTATION_ANGLE = 150;

            Pose2d startPos = new Pose2d(10, 60, Math.toRadians(90));
            snappy.setPoseEstimate(startPos);
            telemetry.update();
            double angles[] = ik.getAngles(EXTENSION_READY_DISTANCE, EXTENSION_READY_HEIGHT);

            snappy.BaseArm.setTargetPosition((int) ((angles[0] - snappy.INITIAL_ARM1_ANGLE) * snappy.ENCODER_TICKS_PER_DEGREE_ARM1));
            //arm.IntakeArm.setTargetPosition((int) ((angles[1] - arm.INITIAL_ARM2_ANGLE + (angles[0] - arm.INITIAL_ARM1_ANGLE) / arm.GEAR_RATIO_ARM2_STAGE) * arm.ENCODER_TICKS_PER_DEGREE_ARM2));
            snappy.IntakeArm.setTargetPosition((int) ((angles[1] - snappy.INITIAL_ARM2_ANGLE) * snappy.ENCODER_TICKS_PER_DEGREE_ARM2));
snappy.ClawServo.setPosition(0.3);
cam.runOpMode(this,true, SnappyHardware.TeamColor.RED);
int position = 3;
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
position = 3; //TAKE THIS OUT LATER?!
            //position = cam.getPosition(); PUT THIS BACK IN LATER!!!!
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

            TrajectorySequence trajectory1 = snappy.trajectorySequenceBuilder(startPos)
                    .back(35)
//                    .turn(Math.toRadians(90))
//                    .strafeRight(5)
//                    .back(35)
                    .build();


            snappy.deliverXblocks(this,123,position);
            snappy.followTrajectorySequence(trajectory1);
            snappy.setArmAnglesToHome(this);

//            telemetry.addData("Height", snappy.height);
//            telemetry.addData("Distance", snappy.distance);
//            telemetry.addData("Rotation", snappy.rotation);
//            telemetry.update();
            //drive foward
           // pickUpBlock(snappy);

//            telemetry.addData("Height", snappy.height);
//            telemetry.addData("Distance", snappy.distance);
//            telemetry.addData("Rotation", snappy.rotation);
//            telemetry.update();
            //drive back
//            snappy.deliverXblocks(this,123,position);
//
//            pickUpBlock(snappy);
//
//            snappy.deliverXblocks(this,123,position);

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

        private void pickUpBlock(SnappyHardware snappy) {
            snappy.StepBreakMovement(this, 0, 126, 40, 1, (long)1);
            snappy.StepBreakMovement(this, 0, 126, -75, .4,(long)1);
            snappy.IntakeServo.setPosition(1);
            snappy.wait(this, 1);
            snappy.IntakeServo.setPosition(0.5);
            snappy.StepBreakMovement(this, 0, 126, 40, 1, (long)1);
            snappy.StepBreakMovement(this, snappy.INITIAL_ROTATION_ANGLE,91,-10,1,1);
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






