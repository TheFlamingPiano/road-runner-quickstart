
    package org.firstinspires.ftc.teamcode.drive.opmode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.CyrusCarouselHardware;
import org.firstinspires.ftc.teamcode.drive.CyrusIntakeArmHardware;
import org.firstinspires.ftc.teamcode.drive.CyrusOfficialHardware;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


    /*
     * This is an example of a more complex path to really test the tuning.
     */

@Disabled
    @Autonomous(group = "drive",name="Test Auto")
    public class TestAuto extends LinearOpMode {
        final double EXTENSION_READY_DISTANCE = 0.0;
        final double EXTENSION_READY_HEIGHT = 55.0;


        CyrusIntakeArmHardware ik;

        @Override
        public void runOpMode() throws InterruptedException {
          DcMotorEx  leftFront  = hardwareMap.get(DcMotorEx.class, "LeftFront");  // Hub 1, Port 0
            DcMotorEx leftRear = hardwareMap.get(DcMotorEx.class, "LeftBack");  // Hub 1, port 1
            DcMotorEx rightRear = hardwareMap.get(DcMotorEx.class, "RightBack");  // Hub 1, port 2
            DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "RightFront");  // Hub 1, port 3

            CyrusOfficialHardware drive = new CyrusOfficialHardware(hardwareMap);
            CyrusIntakeArmHardware arm = new CyrusIntakeArmHardware(hardwareMap, true);
            CyrusCarouselHardware duck = new CyrusCarouselHardware(hardwareMap);
            ik = new CyrusIntakeArmHardware(arm.ARM1_LENGTH, arm.ARM2_LENGTH);
            waitForStart();
while (!isStopRequested())  {
    telemetry.addData("LeftFront", leftFront.getCurrentPosition());
                telemetry.addData("LeftBack", leftRear.getCurrentPosition());
                telemetry.addData("RightBack", rightRear.getCurrentPosition());
                telemetry.addData("RightFront", rightFront.getCurrentPosition());
telemetry.update();

leftFront.setPower(.5);
            }

            if (isStopRequested()) return;

            // move arms to the "ready" position
            double angles[] = ik.getAngles(EXTENSION_READY_DISTANCE, EXTENSION_READY_HEIGHT);

            arm.BaseArm.setTargetPosition((int) ((angles[0] - arm.INITIAL_ARM1_ANGLE) * arm.ENCODER_TICKS_PER_DEGREE_ARM1));
            //arm.IntakeArm.setTargetPosition((int) ((angles[1] - arm.INITIAL_ARM2_ANGLE + (angles[0] - arm.INITIAL_ARM1_ANGLE) / arm.GEAR_RATIO_ARM2_STAGE) * arm.ENCODER_TICKS_PER_DEGREE_ARM2));
            arm.IntakeArm.setTargetPosition((int) ((angles[1] - arm.INITIAL_ARM2_ANGLE) * arm.ENCODER_TICKS_PER_DEGREE_ARM2));

            telemetry.addData("angle 1", angles[0]);
            telemetry.addData("angle 2", angles[1]);
            telemetry.update();

            arm.BaseArm.setPower(0.5);
            arm.IntakeArm.setPower(0.5);
            // wait for 5 seconds and stop
            sleep(5000);

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






