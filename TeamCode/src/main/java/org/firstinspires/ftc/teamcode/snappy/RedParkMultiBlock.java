
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
            SnappyHardware snappy = new SnappyHardware(hardwareMap,true, SnappyHardware.TeamColor.RED);
            CameraSnap cam = new CameraSnap();

            ik = new CyrusIntakeArmHardware(snappy.ARM1_LENGTH, snappy.ARM2_LENGTH);


            Pose2d startPos = new Pose2d(10, -60, Math.toRadians(180));
            Pose2d startMod = new Pose2d(10,-61.4,Math.toRadians(180));
            Pose2d warehousePos = new Pose2d(29, -60, Math.toRadians(180));
            snappy.setPoseEstimate(startPos);
            telemetry.update();
            double angles[] = ik.getAngles(EXTENSION_READY_DISTANCE, EXTENSION_READY_HEIGHT);

            snappy.BaseArm.setTargetPosition((int) ((angles[0] - snappy.INITIAL_ARM1_ANGLE) * snappy.ENCODER_TICKS_PER_DEGREE_ARM1));
            //arm.IntakeArm.setTargetPosition((int) ((angles[1] - arm.INITIAL_ARM2_ANGLE + (angles[0] - arm.INITIAL_ARM1_ANGLE) / arm.GEAR_RATIO_ARM2_STAGE) * arm.ENCODER_TICKS_PER_DEGREE_ARM2));
            snappy.IntakeArm.setTargetPosition((int) ((angles[1] - snappy.INITIAL_ARM2_ANGLE) * snappy.ENCODER_TICKS_PER_DEGREE_ARM2));
snappy.ClawServo.setPosition(0.1);
cam.runOpMode(this,true, SnappyHardware.TeamColor.RED);


            waitForStart();
            snappy.moveEncoderWheelDown();

            int position = cam.getPosition(); //TAKE THIS OUT LATER?!
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


            snappy.deliverXblocks(this,119,position, 10);
            for (int i = 0; i < 3; i++) {
                snappy.deliverExtraBlock(SnappyHardware.TeamColor.RED, -i, this, startPos, warehousePos, startMod);
            }
            TrajectorySequence trajectory1 = snappy.trajectorySequenceBuilder(startPos)
                    .back(31)
                    .build();

            snappy.followTrajectorySequence(trajectory1);


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
