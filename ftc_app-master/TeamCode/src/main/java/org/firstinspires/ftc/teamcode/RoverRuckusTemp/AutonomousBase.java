package org.firstinspires.ftc.teamcode.RoverRuckusTemp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

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
        rfDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lfDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rbDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lbDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (opModeIsActive() && rfDrive.getMode() != DcMotor.RunMode.RUN_USING_ENCODER && lfDrive.getMode() != DcMotor.RunMode.RUN_USING_ENCODER){}
    }
    protected void wait(double seconds) {
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
        while(opModeIsActive() && getleftAbs() <= targetPosition && getRightAbs() <= targetPosition) {
            if (!hasCorrected && getleftAbs() >= (targetPosition / 2)) { // Slowing down once we reach 1/2 to our target
                double newPower;
                if (power > 0) {
                    newPower = power - (power / 2);
                } else {
                    newPower = power + (power / 2);
                }
                drive(newPower);
                hasCorrected = true;
            }
        }
        stopDrive();
        telemetry.addLine("Drove " + inches + " inches to target");
        telemetry.update();
    }

    protected void turnHeading(double power, int degreeTarget){ //turns the robot at a set speed to a given z-axis value
        resetAngle();
        heading = getAngle();

        while (opModeIsActive() && Math.abs(heading - degreeTarget) > degreeTarget/2){
            if (heading > degreeTarget){
                setDrivePower(-power, power, -power, power);
            }
            if (heading < degreeTarget){
                setDrivePower(power, -power, power, -power);
            }
            heading = getAngle();
        }
        while (opModeIsActive() && Math.abs(heading - degreeTarget) > 2){
            if (heading > degreeTarget){
                setDrivePower(-power/2,power/2, -power/2, power/2);
            }
            if (heading < degreeTarget){
                setDrivePower(power/2, -power/2, power/2, -power/2);
            }
            heading = getAngle();
        }
        stopDrive();
        telemetry.addLine("Turned " + degreeTarget + "degrees to target");
        telemetry.update();
        resetAngle();
    }

    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    private int getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return (int)globalAngle;
    }

}
