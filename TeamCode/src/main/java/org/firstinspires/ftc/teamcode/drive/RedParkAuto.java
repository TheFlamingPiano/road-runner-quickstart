package org.firstinspires.ftc.teamcode.drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Disabled
@Autonomous(group = "drive")
public class RedParkAuto extends LinearOpMode{
    final double EXTENSION_READY_DISTANCE = 0.0;
    final double EXTENSION_READY_HEIGHT = 55.0;
    CyrusIntakeArmHardware ik;

    @Override
    public void runOpMode() throws InterruptedException {
        CyrusOfficialHardware drive = new CyrusOfficialHardware(hardwareMap);
        CyrusIntakeArmHardware arm = new CyrusIntakeArmHardware(hardwareMap, true);
        CyrusCarouselHardware duck = new CyrusCarouselHardware(hardwareMap);

        ik = new CyrusIntakeArmHardware(arm.ARM1_LENGTH, arm.ARM2_LENGTH);


        Pose2d startPos = new Pose2d(10, 60, Math.toRadians(90));
        drive.setPoseEstimate(startPos);
        telemetry.update();
        double angles[] = ik.getAngles(EXTENSION_READY_DISTANCE, EXTENSION_READY_HEIGHT);

        arm.BaseArm.setTargetPosition((int) ((angles[0] - arm.INITIAL_ARM1_ANGLE) * arm.ENCODER_TICKS_PER_DEGREE_ARM1));
        //arm.IntakeArm.setTargetPosition((int) ((angles[1] - arm.INITIAL_ARM2_ANGLE + (angles[0] - arm.INITIAL_ARM1_ANGLE) / arm.GEAR_RATIO_ARM2_STAGE) * arm.ENCODER_TICKS_PER_DEGREE_ARM2));
        arm.IntakeArm.setTargetPosition((int) ((angles[1] - arm.INITIAL_ARM2_ANGLE) * arm.ENCODER_TICKS_PER_DEGREE_ARM2));
arm.DumpDoor.setPosition(0.3);
        waitForStart();
        arm.BaseArm.setPower(0.5);
        arm.IntakeArm.setPower(0.5);
        if (isStopRequested()) return;

        TrajectorySequence trajectory1 = drive.trajectorySequenceBuilder(startPos)
                .back(4)
                .turn(Math.toRadians(-90))
                .strafeLeft(10)
                .back(30)
                .build();


        drive.followTrajectorySequence(trajectory1);

    }
}
