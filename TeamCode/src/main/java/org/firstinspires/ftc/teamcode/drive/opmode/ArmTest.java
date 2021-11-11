package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp(group = "drive")
public class ArmTest extends LinearOpMode {

    double height = 0.0;

    final double ARM_HEIGHT_VELOCITY = 10; //milimetters per second

    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();

        long currentTime = System.currentTimeMillis();
        long lastTime = currentTime;

        while (opModeIsActive()) {

            currentTime = System.currentTimeMillis();
            long deltaTime = currentTime - lastTime;
            lastTime = currentTime;

            height = height + ARM_HEIGHT_VELOCITY * deltaTime/1000 * (-gamepad1.left_stick_y);

            telemetry.addData("Height = ",height);
            telemetry.update();

        }

    }
}
