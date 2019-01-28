package org.firstinspires.ftc.teamcode.RoverRuckusV2.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RoverRuckusTemp.Base.RobotRunType;
import org.firstinspires.ftc.teamcode.RoverRuckusV2.Base.AutonomousBaseV2;

/**
 * Created by isaac.blandin on 1/21/19.
 */

@Autonomous(name = "crater V2")
public class CraterAutoV2 extends AutonomousBaseV2 {

    public void runOpMode(){

        initRobotV2(RobotRunType.AUTONOMOUS);
        initVuforia();
        initTfod();

        resetEncoders();
        resetArmEncoders();

        waitForStart();

        drive(0.5, 9.5);
        turnHeading(0.25, 90);
        turnHeading(0.15, 90);
        drive(-0.3, 3.9);
        waitSec(1);
        getGoldPos();
        waitSec(0.3);

        if (gold_position == GOLD_POSITION.RIGHT){
            drive(-0.3, 10);
            waitSec(0.5);
            strafeRot(0.5, 1.5);
            strafeRot(-0.5, 1);
            turnHeading(0.3, 90);
            turnHeading(0.2, 90);
            waitSec(0.3);
            drive(0.3, 27);


        } else if (gold_position == GOLD_POSITION.CENTER){
            drive(0.3, 5);
            waitSec(0.5);
            strafeRot(0.5, 1.5);
            strafeRot(-0.5,1 );
            turnHeading(0.3, 90);
            turnHeading(0.2, 90);
            waitSec(0.3);
            drive(0.3, 12);

        } else {
            drive(0.3, 17);
            waitSec(0.5);
            strafeRot(0.5, 1.5);
            strafeRot(-0.5, 1);
            turnHeading(0.3, 90);
            turnHeading(0.2, 90);

        }

        waitSec(0.3);
        drive(0.5, 32);
        waitSec(0.3);
        turnHeading(0.3, 135);
        turnHeading(0.2, 135);
        waitSec(0.3);
        drive(0.5, 30);
        waitSec(0.2);
        collectorTime(1, -1);
        drive(-0.5, 30);
        waitSec(0.3);
        strafeRot(-0.5, 0.5);
        waitSec(0.3);
        turnHeading(0.45, -42);
        turnHeading(0.2, -42);
        waitSec(0.3);
        strafeRot(-0.5, 0.5);
        waitSec(0.3);
        drive(0.5, 20);
        waitSec(0.3);
        extendArm(10, 1);

        stop();

    }
}
