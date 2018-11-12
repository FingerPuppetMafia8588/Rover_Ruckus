package org.firstinspires.ftc.teamcode.RoverRuckusTemp.HardwareTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Created by isaac.blandin on 11/5/18.
 */

@TeleOp(name = "Color Test")
public class ColorSensorSampling extends LinearOpMode {

    protected ColorSensor color;

    @Override public void runOpMode(){
        color = hardwareMap.get(ColorSensor.class, "color");

        waitForStart();

        while(opModeIsActive()){
                telemetry.addData("Red", color.red());
                telemetry.addData("Blue", color.blue());
                telemetry.addData("Green", color.green());
                telemetry.update();

        }
        stop();
    }

}
