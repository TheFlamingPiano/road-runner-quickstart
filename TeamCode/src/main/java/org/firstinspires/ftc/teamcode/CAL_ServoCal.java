package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp(name = "CAL_ServoCal", group = "CALIBRATION")
//@Disabled
public class CAL_ServoCal extends LinearOpMode {

    //An Analog Input.
    AnalogInput pot;
    double voltage;
    double maxVoltage;
    double servoVal;
    Servo servo0;
    Servo servo1;
    double servoValPrevious = 0.5;
    double windowSize = 0.001;
    //CDI. Using this, we can read any analog sensor on this CDI without creating an instance for each sensor.
    //DeviceInterfaceModule cdi;

    @Override
    public void runOpMode() {
        //Link objects to configuration file
        pot = hardwareMap.analogInput.get("pot");
        servo0 = hardwareMap.servo.get("servo0");
        servo1 = hardwareMap.servo.get("servo1");
        servo0.setPosition(0.5);
        servo1.setPosition(0.5);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            //Read the potentiometer sensor using the Analog Input object
            voltage = pot.getVoltage();
            maxVoltage = pot.getMaxVoltage();
            //servoVal = Range.scale(voltage,0.25,4.75,0,1);
            servoVal = voltage / maxVoltage;
            if ((servoVal > servoValPrevious + windowSize) || (servoVal < servoValPrevious - windowSize)) {
                servo0.setPosition(servoVal);
                servo1.setPosition(servoVal);
                servoValPrevious = servoVal;
            }
            telemetry.addData("Pot Voltage ", "%.2f Volts",  voltage);
            telemetry.addData("Max Voltage ", "%.1f Volts", maxVoltage);
            telemetry.addData("Servo Setpoint ", "%.3f", servoValPrevious);
            telemetry.update();
        }
    }
}