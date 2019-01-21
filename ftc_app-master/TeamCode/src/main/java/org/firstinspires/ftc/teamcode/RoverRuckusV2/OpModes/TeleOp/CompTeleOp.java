package org.firstinspires.ftc.teamcode.RoverRuckusV2.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RoverRuckusTemp.Base.RobotHardware;
import org.firstinspires.ftc.teamcode.RoverRuckusTemp.Base.RobotRunType;
import org.firstinspires.ftc.teamcode.RoverRuckusV2.Base.RoverHardwareV2;

/**
 * Created by isaac.blandin on 1/20/19.
 */

@TeleOp(name = "TeleOp V2")
public class CompTeleOp extends RoverHardwareV2{

    @Override
    public void runOpMode() {

        initRobotV2(RobotRunType.TELEOP);

        double collectorPow;

        waitForStart();
        while(opModeIsActive()){


            /** //////////////////////////////////
             ///////////////Driver 1///////////////
             ////////////////////////////////// **/

            //control drive train
            FieldCentricDrive();

            /** //////////////////////////////////
             ///////////////Driver 1///////////////
             ////////////////////////////////// **/

            armRight.setPower(-gamepad2.left_stick_y);
            armLeft.setPower(-gamepad2.left_stick_y);

            armExtension.setPower(-gamepad2.right_stick_y);

            if (gamepad2.right_bumper){
                collectorPow = 1;
            } else if (gamepad2.left_bumper){
                collectorPow = -1;
            } else {
                collectorPow = 0;
            }

            collector.setPower(collectorPow);

        }
        stop();
    }

}
