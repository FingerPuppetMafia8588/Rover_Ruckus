package org.firstinspires.ftc.teamcode.RoverRuckusTemp.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.RoverRuckusTemp.Base.AutonomousBase;
import org.firstinspires.ftc.teamcode.RoverRuckusTemp.Base.RobotRunType;

/**
 * Created by isaac.blandin on 11/6/18.
 *
 * An autonomous Program for the crater facing side of the lander which samples the gold and parks in the crater
 */

@Autonomous(name = "comp auto crater", group = "Autonomous")
@Disabled
public class RoverAutoCrater extends AutonomousBase {
    @Override public void runOpMode(){
        initRobot(RobotRunType.AUTONOMOUS);

        waitForStart();

        resetEncoders();
        //drives to minerals
        drive(.15, 19);
        waitSec(1);

        //realigns robot
        turnHeading(0.15, 0);
        waitSec(0.5);
        turnHeading(0.15, 0);
        waitSec(1);

        //samples gold
        sample();

        //drives to crater
        waitSec(.5);
        drive(0.15, 6);

    }
}
