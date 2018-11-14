package org.firstinspires.ftc.teamcode.RoverRuckusTemp.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.RoverRuckusTemp.Base.AutonomousBase;
import org.firstinspires.ftc.teamcode.RoverRuckusTemp.Base.RobotRunType;

/**
 * Created by isaac.blandin on 11/8/18.
 *
 * An autonomous program for the box facing side of the lander which samples the gold, drops the team marker,
 * and parks in the crater
 */

@Autonomous(name = "comp auto box", group = "Autonomous")
public class RoverAutoBox extends AutonomousBase {

    @Override public void runOpMode() {
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
        drive(.15, 31);
        waitSec(1);
        turnHeading(0.2, -150);

        marker.setPosition(1);
        waitSec(.5);

        turnHeading(0.2, -135);
        waitSec(0.5);
        strafeRot(-0.3, 1);
        waitSec(0.5);
        turnHeading(0.2, -135);
        waitSec(0.5);
        drive(0.4, 55);

    }
}
