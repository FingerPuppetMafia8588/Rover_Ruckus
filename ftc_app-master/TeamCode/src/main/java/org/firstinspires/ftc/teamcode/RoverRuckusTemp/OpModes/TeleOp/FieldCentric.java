package org.firstinspires.ftc.teamcode.RoverRuckusTemp.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RoverRuckusTemp.Base.AutonomousBase;
import org.firstinspires.ftc.teamcode.RoverRuckusTemp.Base.RobotHardware;
import org.firstinspires.ftc.teamcode.RoverRuckusTemp.Base.RobotRunType;

/**
 * Created by isaac.blandin on 10/19/18.
 */

@TeleOp(name = "field centric")
public class FieldCentric extends AutonomousBase{

    public void runOpMode(){
        initRobot(RobotRunType.AUTONOMOUS);

        waitForStart();
        while (opModeIsActive()){


            if(gamepad1.a && gamepad1.b){
                resetAngle();
            }

            double x1 = gamepad1.left_stick_x, y1 = -gamepad1.left_stick_y;
            double v = Math.sqrt(x1 * x1 + y1 * y1);
            double theta = Math.atan2(x1, y1);
            double current = Math.toRadians(getAngle() % 360);
            drive(theta + current, v, gamepad1.right_stick_x);
        }
        stop();
    }

    private Wheels getWheels(double direction, double velocity, double rotationVelocity) {
        final double vd = velocity;
        final double td = direction;
        final double vt = rotationVelocity;

        double s = Math.sin(td + Math.PI / 4.0);
        double c = Math.cos(td + Math.PI / 4.0);
        double m = Math.max(Math.abs(s), Math.abs(c));
        s /= m;
        c /= m;

        final double v1 = vd * s + vt;
        final double v2 = vd * c - vt;
        final double v3 = vd * c + vt;
        final double v4 = vd * s - vt;

        // Ensure that none of the values go over 1.0. If none of the provided values are
        // over 1.0, just scale by 1.0 and keep all values.
        double scale = ma(1.0, v1, v2, v3, v4);

        return new Wheels(v1 / scale, v2 / scale, v3 / scale, v4 / scale);
    }

    private static double ma(double... xs) {
        double ret = 0.0;
        for (double x : xs) {
            ret = Math.max(ret, Math.abs(x));
        }
        return ret;
    }

    private void drive(double direction, double velocity, double rotationVelocity) {
        Wheels w = getWheels(direction, velocity, rotationVelocity);
        lfDrive.setPower(w.lf);
        rfDrive.setPower(w.rf);
        lbDrive.setPower(w.lr);
        rbDrive.setPower(w.rr);
    }


}


