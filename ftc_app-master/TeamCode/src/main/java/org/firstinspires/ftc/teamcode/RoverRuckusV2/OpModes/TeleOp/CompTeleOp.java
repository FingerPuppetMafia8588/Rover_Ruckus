package org.firstinspires.ftc.teamcode.RoverRuckusV2.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.RoverRuckusTemp.Base.RobotRunType;
import org.firstinspires.ftc.teamcode.RoverRuckusV2.Base.AutonomousBaseV2;
/**
 * Created by isaac.blandin on 1/20/19.
 */

@TeleOp(name = "TeleOp V2")
public class CompTeleOp extends AutonomousBaseV2{

    @Override
    public void runOpMode() {

        //set up robot for teleop
        initRobotV2(RobotRunType.TELEOP);
        double collectorPow;

        while(!opModeIsActive() && !isStopRequested()){
            telemetry.addData("status", "waiting for start command...");
        }
        while(opModeIsActive()){


            /** //////////////////////////////////
             ///////////////Driver 1///////////////
             ////////////////////////////////// **/

            //control drive train
            FieldCentricDrive();
            //allows reset of gyroscope when aligned with driver preference
            if (gamepad1.a && gamepad1.b){
                resetAngle();
            }

            /** //////////////////////////////////
             ///////////////Driver 1///////////////
             ////////////////////////////////// **/

            //control rotation of arm
            armRight.setPower(-gamepad2.left_stick_y);
            armLeft.setPower(-gamepad2.left_stick_y);
            //control extension of arm
            armExtension.setPower(-gamepad2.right_stick_y);
            //control the collector
            if (gamepad2.right_bumper){
                collectorPow = 1;
            } else if (gamepad2.left_bumper){
                collectorPow = -1;
            } else {
                collectorPow = 0;
            }
            collector.setPower(collectorPow);
            //control dumping of minerals
            if (gamepad2.a){
                dump.setPosition(0.4);
            }  else{
                dump.setPosition(1);
            }

        }
        stop();
    }

}
