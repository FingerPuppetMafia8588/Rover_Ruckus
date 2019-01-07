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

            //handle gamepad
            x1 = gamepad1.left_stick_x * STRAFE_RATIO;
            y1 = -gamepad1.left_stick_y * FORWARD_RATIO;
            x2 = gamepad1.right_stick_x * TURN_RATIO;

            MecanumFormula(y1, x1, x2);

            ////////////////////////////////////
            //////////////Driver 2//////////////
            ////////////////////////////////////

            //use left joystick to raise and lower main linear slide
            liftL.setPower(gamepad2.left_stick_y);
            liftR.setPower(gamepad2.left_stick_y);

            hang.setPower(gamepad2.right_stick_y);

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
                collectorL.setPosition(0.45);
                collectorR.setPosition(0.55);
            }
            if (gamepad2.dpad_down){
                collectorL.setPosition(0.88);
                collectorR.setPosition(0.12);
            }

        }
    }
}
