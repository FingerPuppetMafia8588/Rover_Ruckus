package org.firstinspires.ftc.teamcode.RoverRuckusTemp.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.RoverRuckusTemp.Base.AutonomousBase;
import org.firstinspires.ftc.teamcode.RoverRuckusTemp.Base.RobotRunType;

/**
 * Created by isaac.blandin on 11/6/18.
 */

@Autonomous(name = "comp auto crater", group = "Autonomous")
public class RoverAutoCrater extends AutonomousBase {
    @Override public void runOpMode(){
        initRobot(RobotRunType.AUTONOMOUS);
        
        waitForStart();

        resetEncoders();
        drive(.15, 19);
        waitSec(1);

        turnHeading(0.15, 0);
        waitSec(0.5);
        turnHeading(0.15, 0);
        waitSec(1);

        sample();

        waitSec(.5);
        drive(0.15, 6);

    }
}
