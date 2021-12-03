package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.snappy.SnappyHardware;

import java.util.Objects;

/**
 * This routine is designed to calculate the maximum angular velocity your bot can achieve under load.
 * <p>
 * Upon pressing start, your bot will turn at max power for RUNTIME seconds.
 * <p>
 * Further fine tuning of MAX_ANG_VEL may be desired.
 */

@Config
@Disabled
@Autonomous(group = "drive")
public class MaxAngularVeloTuner extends LinearOpMode {
    public static double RUNTIME = 4.0;

    private ElapsedTime timer;
    private double maxAngVelocity = 0.0;
    private double avgAngVelocity = 0.0;   // aml added this
    private int avgCount = 0;              // aml added this
    private double curVel = 0.0;           // aml added this

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        SnappyHardware snappy = new SnappyHardware(hardwareMap, true);


        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("Your bot will turn at full speed for " + RUNTIME + " seconds.");
        telemetry.addLine("Please ensure you have enough space cleared.");
        telemetry.addLine("");
        telemetry.addLine("Press start when ready.");
        telemetry.update();

        waitForStart();

        telemetry.clearAll();
        telemetry.update();

        drive.setDrivePower(new Pose2d(0, 0, 1));
        timer = new ElapsedTime();

        while (!isStopRequested() && timer.seconds() < RUNTIME) {
            drive.updatePoseEstimate();

            Pose2d poseVelo = Objects.requireNonNull(drive.getPoseVelocity(), "poseVelocity() must not be null. Ensure that the getWheelVelocities() method has been overridden in your localizer.");

            curVel = poseVelo.getHeading();
            maxAngVelocity = Math.max(curVel, maxAngVelocity);
            //maxAngVelocity = Math.max(poseVelo.getHeading(), maxAngVelocity);
            avgAngVelocity += curVel;
            avgCount++;
        }

        snappy.RotationMotor.setTargetPosition(0);
        snappy.RotationMotor.setPower(1);
        drive.setDrivePower(new Pose2d());

        telemetry.addData("Max Angular Velocity (rad)", maxAngVelocity);
        telemetry.addData("Max Angular Velocity (deg)", Math.toDegrees(maxAngVelocity));
        telemetry.addData("Average Velocity", avgAngVelocity / avgCount);
        telemetry.update();

        while (!isStopRequested()) idle();
    }
}
