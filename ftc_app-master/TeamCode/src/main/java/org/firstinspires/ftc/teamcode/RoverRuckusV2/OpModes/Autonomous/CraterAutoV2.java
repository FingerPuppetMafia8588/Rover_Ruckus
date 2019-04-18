package org.firstinspires.ftc.teamcode.RoverRuckusV2.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RoverRuckusTemp.Base.RobotRunType;
import org.firstinspires.ftc.teamcode.RoverRuckusV2.Base.AutoType;
import org.firstinspires.ftc.teamcode.RoverRuckusV2.Base.AutonomousBaseV2;

/**
 * Created by isaac.blandin on 1/21/19.
 */

@Autonomous(name = "crater full")
public class CraterAutoV2 extends AutonomousBaseV2 {

    public void runOpMode(){

        //set up robot
        initRobotV2(RobotRunType.AUTONOMOUS);
        initVuforia();
        initTfod();
        //free motors for landing
        armRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //reset all encoders
        resetEncoders();
        resetArmEncoders();

        while(!opModeIsActive() && !isStopRequested()){
            telemetry.addData("status", "waiting for start command...");
        }

        //land robot
        //land();
        craterClaim();
        //sample minerals
        //sample(AutoType.CRATER);

        //extendArm(5,1);
        /**
        //drive to depot
        waitSec(0.3);
        drive(0.5, 36);
        waitSec(0.3);
        turnHeading(0.3, 125);
        turnHeading(0.2, 125);
        waitSec(0.3);
        drive(0.5, 30);
        waitSec(0.2);
        //drop the team marker
        collectorTime(1, -1);
        //drive to crater to park
        drive(-0.5, 30);
        waitSec(0.2);
        turnHeading(0.3, 145);
        drive(-0.3, 30);

         */
        stop();

    }
}
