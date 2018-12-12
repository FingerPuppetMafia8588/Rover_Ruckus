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

            double x1 = gamepad1.left_stick_x, y1 = -gamepad1.left_stick_y;
            double v = Math.sqrt(x1 * x1 + y1 * y1);
            double theta = Math.atan2(x1, y1);
            double current = Math.toRadians(getGlobal() % 360);
            drive(theta + current, v, gamepad1.right_stick_x);
        }
        stop();
    }




}


