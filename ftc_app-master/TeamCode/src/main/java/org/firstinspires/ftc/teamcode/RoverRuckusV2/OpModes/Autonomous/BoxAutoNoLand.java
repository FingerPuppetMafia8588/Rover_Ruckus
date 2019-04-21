package org.firstinspires.ftc.teamcode.RoverRuckusV2.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RoverRuckusTemp.Base.RobotRunType;
import org.firstinspires.ftc.teamcode.RoverRuckusV2.Base.AutoType;
import org.firstinspires.ftc.teamcode.RoverRuckusV2.Base.AutonomousBaseV2;

/**
 * Created by isaac.blandin on 2/11/19.
 */

@Autonomous(name = "box no land")
public class BoxAutoNoLand extends AutonomousBaseV2{

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

        //sample minerals
        sample(AutoType.DEPOT);

        stop();

    }

}
