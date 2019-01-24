package org.firstinspires.ftc.teamcode.RoverRuckusV2.OpModes.Autonomous;

import org.firstinspires.ftc.teamcode.RoverRuckusTemp.Base.RobotRunType;
import org.firstinspires.ftc.teamcode.RoverRuckusV2.Base.AutonomousBaseV2;

/**
 * Created by isaac.blandin on 1/21/19.
 */

public class BoxAutoV2 extends AutonomousBaseV2{

    public void runOpMode(){

        initRobotV2(RobotRunType.AUTONOMOUS);

        resetEncoders();
        resetArmEncoders();

        waitForStart();

        land();


        stop();
    }
}
