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
public class RedDuck extends LinearOpMode {
    final double EXTENSION_READY_DISTANCE = 0.0;
    final double EXTENSION_READY_HEIGHT = 55.0;

    CyrusIntakeArmHardware ik;
    @Override

    public void runOpMode() throws InterruptedException {

        SnappyHardware snappy = new SnappyHardware(hardwareMap,true, SnappyHardware.TeamColor.RED);
        CameraSnap cam = new CameraSnap();

        ik = new CyrusIntakeArmHardware(snappy.ARM1_LENGTH, snappy.ARM2_LENGTH);

        Pose2d startPos = new Pose2d(-35, 60, Math.toRadians(180));
        snappy.setPoseEstimate(startPos);
        telemetry.update();

        double angles[] = ik.getAngles(EXTENSION_READY_DISTANCE, EXTENSION_READY_HEIGHT);


        snappy.BaseArm.setTargetPosition((int) ((angles[0] - snappy.INITIAL_ARM1_ANGLE) * snappy.ENCODER_TICKS_PER_DEGREE_ARM1));
        //arm.IntakeArm.setTargetPosition((int) ((angles[1] - arm.INITIAL_ARM2_ANGLE + (angles[0] - arm.INITIAL_ARM1_ANGLE) / arm.GEAR_RATIO_ARM2_STAGE) * arm.ENCODER_TICKS_PER_DEGREE_ARM2));
        snappy.IntakeArm.setTargetPosition((int) ((angles[1] - snappy.INITIAL_ARM2_ANGLE) * snappy.ENCODER_TICKS_PER_DEGREE_ARM2));
        snappy.ClawServo.setPosition(0.1);
        cam.runOpMode(this,false, SnappyHardware.TeamColor.RED);
        //int position = 3;
        waitForStart();
        snappy.moveEncoderWheelDown();

        int position = cam.getPosition();
        telemetry.addData("LeftGreen",cam.greenleft-cam.redleft-cam.blueleft);
        telemetry.addData("MiddleGreen",cam.greenmiddle-cam.redmiddle-cam.bluemiddle);
        telemetry.addData("RightGreen",cam.greenright-cam.redright-cam.blueright);
        telemetry.addData("Position", position);
        telemetry.update();
        snappy.BaseArm.setPower(1);
        snappy.IntakeArm.setPower(1);
        snappy.RotationMotor.setPower(1);

        snappy.setArmAnglesToHome(this);

        if (isStopRequested()) return;

        TrajectorySequence trajectory1 = snappy.trajectorySequenceBuilder(startPos)
                .strafeRight(8)
                .forward(18)
                .turn(Math.toRadians(45))
                .build();

        TrajectorySequence trajectory2 = snappy.trajectorySequenceBuilder(trajectory1.end())
                .forward(5)
                .build();
        TrajectorySequence trajectory3 = snappy.trajectorySequenceBuilder(trajectory2.end())
                .waitSeconds(4)
                .back(10)
                .build();

        TrajectorySequence trajectory4 = snappy.trajectorySequenceBuilder(trajectory3.end())
                .waitSeconds(2)
                .turn(Math.toRadians(45))
                .back(17)
                .strafeRight(12)
                .build();
//        TrajectorySequence trajectory4 = snappy.trajectorySequenceBuilder(trajectory3.end())
//                .waitSeconds(2)
//                .turn(Math.toRadians(45))
//                .back(17)
//                .turn(Math.toRadians(-90))
//                .forward(12)
//                .build();




        snappy.deliverXblocks(this,55,position, 155);
        snappy.StepBreakMovement(this, 0, 33,-4,70,(long)1);
        snappy.followTrajectorySequence(trajectory1);
        snappy.followTrajectorySequence(trajectory2);
        snappy.RedSpin();
        snappy.followTrajectorySequence(trajectory3);
        snappy.StopCarousel();
        snappy.StepBreakMovement(this, snappy.INITIAL_ROTATION_ANGLE, snappy.INITIAL_DISTANCE, 80, 140,(long)1);

        //0, 50, -21.6, -45, 1
        //this, snappy.INITIAL_ROTATION_ANGLE, snappy.INITIAL_DISTANCE, 80, 140,(long)1
        snappy.followTrajectorySequence(trajectory4);
        //snappy.ClawServo.setPosition(1);

    }
}





