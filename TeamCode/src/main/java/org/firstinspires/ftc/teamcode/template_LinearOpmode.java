package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//@TeleOp(name = "opmode name", group = "your group")
//@Autonomous(name = "opmode name", group = "your group")
@Disabled  // remove this line so opmode shows up in list
public class template_LinearOpmode extends LinearOpMode {

    // Declare OpMode members.



    // Main run loop. Begins when Init pressed.
    @Override
    public void runOpMode() throws InterruptedException {



        // Wait for Start command from Driver Station
        waitForStart();


    }
}
