package org.firstinspires.ftc.teamcode.RoverRuckusTemp.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.RoverRuckusTemp.Base.AutonomousBase;
import org.firstinspires.ftc.teamcode.RoverRuckusTemp.Base.RobotRunType;

/**
 * Created by isaac.blandin on 11/6/18.
 */

@Autonomous(name = "comp auto crater")
public class RoverAutoCrater extends AutonomousBase {
    @Override public void runOpMode(){
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
        armR.setPosition(0.5);
        waitSec(1);

        if (checkGoldL()){
            armR.setPosition(1);
        } else if (checkGoldR()){
            armL.setPosition(0);
        } else {
            armL.setPosition(0);
            armR.setPosition(1);

            collectorL.setPosition(0.9);
            collectorR.setPosition(0.1);
        }
        waitSec(1);

        drive(0.15, 6);
        waitSec(0.5);
        armR.setPosition(1);
        armL.setPosition(0);
        collectorL.setPosition(.5);
        collectorR.setPosition(0.5);
        waitSec(.5);
        drive(0.15, 6);

        /*
        drive(-0.15, 10);

        waitSec(1);

        turnHeading(0.15, 0);

        waitSec(1);

        turnHeading(0.15, 90);
        waitSec(0.5);

        drive(0.4, 33);
        waitSec(0.5);
        turnHeading(0.15, 90);
        waitSec(.25);
        resetAngle();
        turnHeading(0.15, 45);
        waitSec(0.5);
        drive(0.4, 35);

        */



    }
}
