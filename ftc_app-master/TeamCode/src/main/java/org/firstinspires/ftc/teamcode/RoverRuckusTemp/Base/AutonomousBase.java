package org.firstinspires.ftc.teamcode.RoverRuckusTemp.Base;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by isaac.blandin on 8/28/18.
 */

public abstract class AutonomousBase extends RobotHardware {

    protected void resetEncoders(){
        rfDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lfDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rbDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lbDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while (opModeIsActive() && rfDrive.getCurrentPosition() > 3 && lfDrive.getCurrentPosition() > 3){}
        rfDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lfDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rbDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lbDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (opModeIsActive() && rfDrive.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER && lfDrive.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER){}
    }
    protected void waitSec(double seconds) {
        ElapsedTime t = new ElapsedTime(System.nanoTime());
        while (opModeIsActive() && t.time() <= seconds) {

        }
    }
    protected int getRightAbs(){return Math.abs(rfDrive.getCurrentPosition());}
    protected int getleftAbs(){return Math.abs(lfDrive.getCurrentPosition());}

    ///////////////////////////////////
    /////////////Movement//////////////
    ///////////////////////////////////

    protected void drive(double power){
        setDrivePower(power,power, power, power);
    }

    protected void drive(double power, double inches){ // Moves the robot in a straight line at a determined power and distance
        resetEncoders();
        drive(power);
        double targetPosition = inches*ORBITAL20_PPR*DRIVE_GEAR_RATIO/WHEEL_CIRC;
        boolean hasCorrected = false;
        double starting = getAngle();
        double right = power;
        double left = power;
        while(opModeIsActive() && getleftAbs() <= targetPosition && getRightAbs() <= targetPosition) {
            if (getAngle() > starting + 1){
                left = power * 1.4;
                right = power;
            } else if (getAngle() < starting - 1){
                right = power * 1.4;
                left = power;
            }
            setDrivePower(right, left, right, left);
        }
        stopDrive();
        telemetry.addLine("Drove " + inches + " inches to target");
        telemetry.update();
    }

    protected void strafeRot(double power, double rotations){
        resetEncoders();
        setDrivePower(-power, power, power, -power);
        //drive(power);
        double targetPosition = rotations*ORBITAL20_PPR*DRIVE_GEAR_RATIO;

        while(opModeIsActive() && getleftAbs() <= targetPosition && getRightAbs() <= targetPosition) {

        }
        stopDrive();
        telemetry.addLine("Strafed " + rotations + " Rotations to target");
        telemetry.update();
    }

    protected void turnHeading(double power, int degreeTarget){ //turns the robot at a set speed to a given z-axis value
        //resetAngle();
        heading = getAngle();

        while (opModeIsActive() && Math.abs(heading - degreeTarget) > 0){
            if (heading > degreeTarget){
                setDrivePower(-power, power, -power, power);
            }
            if (heading < degreeTarget){
                setDrivePower(power, -power, power, -power);
            }
            heading = getAngle();
        }
       /* while (opModeIsActive() && Math.abs(heading - degreeTarget) > 1){
            if (heading > degreeTarget){
                setDrivePower(-power/2,power/2, -power/2, power/2);
            }
            if (heading < degreeTarget){
                setDrivePower(power/2, -power/2, power/2, -power/2);
            }
            heading = getAngle();
        }*/
        stopDrive();
        telemetry.addLine("Turned " + degreeTarget + "degrees to target");
        telemetry.update();
        resetAngle();
    }

    ///////////////////////////////////
    ///////////////Gyro////////////////
    ///////////////////////////////////

    protected void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    protected int getAngle()
    {

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;


        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return (int)angles.firstAngle;
    }



    ///////////////////////////////////
    ///////////////Data////////////////
    ///////////////////////////////////

    protected boolean checkGoldL(){

        if (colorL.red() + colorL.green() > colorL.blue() * 2.6){
            return true;
        } else {
            return false;
        }
    }

    protected boolean checkGoldR(){

        if (colorR.red() + colorR.green() > colorR.blue() * 2.6){
            return true;
        } else {
            return false;
        }
    }

}
