package org.firstinspires.ftc.teamcode.RoverRuckusTemp.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.RoverRuckusTemp.Base.AutonomousBase;
import org.firstinspires.ftc.teamcode.RoverRuckusTemp.Base.RobotRunType;

/**
 * Created by isaac.blandin on 11/8/18.
 */

@Autonomous(name = "comp auto box")
public class RoverAutoBox extends AutonomousBase {

    @Override public void runOpMode() {
        initRobot(RobotRunType.AUTONOMOUS);
        boolean isGold = false;
        waitForStart();

        resetEncoders();
        drive(.15, 19);
        waitSec(1);

        turnHeading(0.15, 0);
        waitSec(0.5);
        turnHeading(0.15, 0);
        waitSec(1);

        armL.setPosition(0.5);
        armR.setPosition(0.55);
        waitSec(1);

        if (checkGoldL()){
            armR.setPosition(1);
        } else if (checkGoldR()){
            armL.setPosition(0);
        } else {
            armL.setPosition(0);
            armR.setPosition(1);

            collector.setPosition(0.9);
        }
        waitSec(1);

        drive(0.15, 6);
        waitSec(0.5);
        armR.setPosition(1);
        armL.setPosition(0);
        collector.setPosition(.5);
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
