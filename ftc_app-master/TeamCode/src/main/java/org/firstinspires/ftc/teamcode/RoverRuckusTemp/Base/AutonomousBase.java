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
        double starting = getAngle();
        double right = power;
        double left = power;
        while(opModeIsActive() && getleftAbs() <= targetPosition && getRightAbs() <= targetPosition) {
            double delta = starting - getAngle();
            right = power + (delta / 40);
            left = power - (delta / 40);

            if (Math.abs(right) > 1 || Math.abs(left) > 1){
                right /= Math.max(right, left);
                left /= Math.max(right, left);
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

        //turn until gyro value is met
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
    ///////////////Tasks///////////////
    ///////////////////////////////////

    protected void sample(){

        //lower scanning arms
        armL.setPosition(0.5);
        armR.setPosition(0.5);
        waitSec(0.5);

        //scan for position of gold
        GoldPosition goldPosition = getGoldPos();

        //lower respective arms for the scanner gold
        if (goldPosition == GoldPosition.LEFT){
            armR.setPosition(1);
        } else if (goldPosition == GoldPosition.RIGHT){
            armL.setPosition(0);
        } else {
            armL.setPosition(0);
            armR.setPosition(1);

            collectorL.setPosition(0.9);
            collectorR.setPosition(0.1);
        }
        waitSec(0.5);

        //drive into gold
        drive(0.15, 8);
        waitSec(0.5);
        armR.setPosition(1);
        armL.setPosition(0);
        collectorL.setPosition(.5);
        collectorR.setPosition(0.5);
    }

    ///////////////////////////////////
    ///////////////Gyro////////////////
    ///////////////////////////////////

    protected void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    private int getAngle()
    {

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return (int)angles.firstAngle;
    }


    ///////////////////////////////////
    ///////////////Data////////////////
    ///////////////////////////////////

    //checks if gold mineral is on left
    protected boolean checkGoldL(){

        if (colorL.red() + colorL.green() > colorL.blue() * GOLD_RATIO){
            return true;
        } else {
            return false;
        }
    }

    //checks if gold mineral is on right
    protected boolean checkGoldR(){

        if (colorR.red() + colorR.green() > colorR.blue() * GOLD_RATIO){
            return true;
        } else {
            return false;
        }
    }

    //gives the position of the gold mineral
    protected GoldPosition getGoldPos(){

        if (checkGoldL()){
            return GoldPosition.LEFT;
        } else if (checkGoldR()){
            return GoldPosition.RIGHT;
        } else {
            return GoldPosition.CENTER;
        }
    }

    protected enum GoldPosition {

        LEFT, CENTER, RIGHT
    }

}
