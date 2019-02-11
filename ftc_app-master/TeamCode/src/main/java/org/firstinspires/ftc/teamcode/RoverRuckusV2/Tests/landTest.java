package org.firstinspires.ftc.teamcode.RoverRuckusV2.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RoverRuckusTemp.Base.RobotRunType;
import org.firstinspires.ftc.teamcode.RoverRuckusV2.Base.AutonomousBaseV2;

/**
 * Created by isaac.blandin on 2/6/19.
 */

@Autonomous(name = "land")
@Disabled
public class landTest extends AutonomousBaseV2 {

    @Override
    public void runOpMode(){
        initRobotV2(RobotRunType.AUTONOMOUS);
        resetArmEncoders();
        resetEncoders();
        armLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        armRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        waitForStart();

        waitSec(1);
        rotArm(2, -1);
        hangLock.setPosition(1);
        waitSec(2);
        armRight.setPower(0.5);
        armLeft.setPower(0.5);
        while(Math.abs(armRight.getCurrentPosition()) < 2){

        }
        armRight.setPower(0);
        armLeft.setPower(0);

        rotArm(5, 1);

        turnHeading(0.3, 10);
        strafeRot(1, 0.4);
        rotArm(75, -1);
        turnHeading(0.3, 0);
        strafeRot(-0.4, 0.23);
        waitSec(0.2);
        drive(-0.5, 3);
        turnHeading(0.3, 0);

    }
}
