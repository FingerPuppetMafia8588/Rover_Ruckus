package org.firstinspires.ftc.teamcode.RoverRuckusTemp.HardwareTests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RoverRuckusTemp.Base.RobotHardware;
import org.firstinspires.ftc.teamcode.RoverRuckusTemp.Base.RobotRunType;

/**
 * Created by isaac.blandin on 11/16/18.
 */

@TeleOp(name = "Encoder Test")
public class RunUsingEncoderTest extends RobotHardware {


    @Override
    public void runOpMode(){

        initRobot(RobotRunType.TELEOP);

        rfDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rbDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lfDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lbDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        while(opModeIsActive()){

            MecanumFormula(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        }
        stop();

    }
}
