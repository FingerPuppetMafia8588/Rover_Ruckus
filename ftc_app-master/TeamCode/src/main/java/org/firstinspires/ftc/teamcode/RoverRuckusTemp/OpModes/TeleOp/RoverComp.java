package org.firstinspires.ftc.teamcode.RoverRuckusTemp.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RoverRuckusTemp.Base.AutonomousBase;
import org.firstinspires.ftc.teamcode.RoverRuckusTemp.Base.RobotHardware;
import org.firstinspires.ftc.teamcode.RoverRuckusTemp.Base.RobotRunType;

/**
 * Created by isaac.blandin on 9/29/18.
 */
@TeleOp(name = "rover comp")
public class RoverComp extends RobotHardware{

    @Override
    public void runOpMode(){
        initRobot(RobotRunType.TELEOP);


        waitForStart();

        double rfPower, rbPower, lfPower, lbPower;

        while (opModeIsActive()){


            ////////////////////////////////////
            //////////////Driver 1//////////////
            ////////////////////////////////////

            double x1;
            double x2;
            double y1;

            double max;

            //handle gamepad
            x1 = gamepad1.left_stick_x * STRAFE_RATIO;
            y1 = -gamepad1.left_stick_y * FORWARD_RATIO;
            x2 = gamepad1.right_stick_x * TURN_RATIO;

            //reset powers
            lfPower = 0;
            lbPower = 0;
            rfPower = 0;
            rbPower = 0;

            //handle forward/backward movement
            lfPower += y1;
            lbPower += y1;
            rfPower += y1;
            rbPower += y1;

            //handle strafing movement
            lfPower += x1;
            lbPower -= x1;
            rfPower -= x1;
            rbPower += x1;

            //handle turning movement
            lfPower += x2;
            lbPower += x2;
            rfPower -= x2;
            rbPower -= x2;

            //scale powers of exceeding those of the motor
            max = Math.abs(lfPower);
            if (Math.abs(lbPower) > max){
                max = Math.abs(lbPower);
            }
            if (Math.abs(rfPower) > max){
                max = Math.abs(rfPower);
            }
            if (Math.abs(rbPower) > max){
                max = Math.abs(rbPower);
            }

            if (max > 1){
                lbPower /= max;
                lfPower /= max;
                rbPower /= max;
                rfPower /= max;
            }

            setDrivePower(rfPower, lfPower, rbPower, lbPower);


            ////////////////////////////////////
            //////////////Driver 2//////////////
            ////////////////////////////////////

            //use left joystick to raise and lower main linear slide
            liftL.setPower(gamepad2.left_stick_y);
            liftR.setPower(gamepad2.left_stick_y);

            //use a and b to store and dump mineral box
            if (gamepad2.a & gamepad2.b){
                dumpL.setPosition(0.55);
            } else if (gamepad2.b){
                dumpL.setPosition(1);
            } else if (gamepad2.a){
                dumpL.setPosition(0.65);
            }

            //use dpad up and down to control collector
            if (gamepad2.dpad_up){
                collectorL.setPosition(0.53);
                collectorR.setPosition(0.47);
            }
            if (gamepad2.dpad_down){
                collectorL.setPosition(0.95);
                collectorR.setPosition(0.05);
            }

        }
    }
}
