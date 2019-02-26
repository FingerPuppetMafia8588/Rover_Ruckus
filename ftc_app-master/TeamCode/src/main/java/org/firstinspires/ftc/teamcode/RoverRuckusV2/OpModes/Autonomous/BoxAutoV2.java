package org.firstinspires.ftc.teamcode.RoverRuckusV2.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RoverRuckusTemp.Base.RobotRunType;
import org.firstinspires.ftc.teamcode.RoverRuckusV2.Base.AutoType;
import org.firstinspires.ftc.teamcode.RoverRuckusV2.Base.AutonomousBaseV2;

/**
 * Created by isaac.blandin on 1/21/19.
 */

@Autonomous(name = "box full")
public class BoxAutoV2 extends AutonomousBaseV2{

    public void runOpMode(){

        //set up robot
        initRobotV2(RobotRunType.AUTONOMOUS);
        initVuforia();
        initTfod();
        //free motors for landing
        armRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        armLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //reset all encoders
        resetEncoders();
        resetArmEncoders();

        while(!opModeIsActive() && !isStopRequested()){
            telemetry.addData("status", "waiting for start command...");
        }

        //land robot
        land();
        //sample minerals
        sample(AutoType.DEPOT);
        //drive to depot
        waitSec(0.2);
        drive(0.5, 31);
        waitSec(0.2);
        turnHeading(0.3, -45);
        waitSec(0.2);
        strafeRot(-0.5, 0.5);
        strafeRot(0.3, 0.2);
        waitSec(0.2);
        turnHeading(0.15, -42);
        drive(0.5, 55);
        //drop team marker
        collectorTime(1, -1);
        //drive to crater to park
        drive(-0.5, 30);
        waitSec(0.2);
        strafeRot(0.5, 0.5);
        waitSec(0.2);
        turnHeading(0.45, 135);
        turnHeading(0.2, 135);
        waitSec(0.2);
        strafeRot(0.5, 0.5);
        waitSec(0.2);
        drive(0.5, 20);
        waitSec(0.2);
        //extend arm over crater
        extendArm(10, 1);

        stop();


    }
}
