package org.firstinspires.ftc.teamcode.RoverRuckusTemp.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RoverRuckusTemp.Base.AutonomousBase;
import org.firstinspires.ftc.teamcode.RoverRuckusTemp.Base.RobotHardware;
import org.firstinspires.ftc.teamcode.RoverRuckusTemp.Base.RobotRunType;

/**
 * Created by isaac.blandin on 10/19/18.
 */

@TeleOp(name = "field centric")
@Disabled
public class FieldCentric extends AutonomousBase{

    public void runOpMode(){
        initRobot(RobotRunType.AUTONOMOUS);

        waitForStart();
        while (opModeIsActive()){

            if(gamepad1.a && gamepad1.b){
                resetAngle();
            }

            FieldCentricDrive();
        }
        stop();
    }




}


