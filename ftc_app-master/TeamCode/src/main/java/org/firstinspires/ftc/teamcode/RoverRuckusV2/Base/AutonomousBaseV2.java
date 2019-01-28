package org.firstinspires.ftc.teamcode.RoverRuckusV2.Base;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.RoverRuckusTemp.Base.AutonomousBase;
import org.firstinspires.ftc.teamcode.RoverRuckusV2.Tests.InferredMIneralDetection;

import java.util.List;

/**
 * Created by isaac.blandin on 1/16/19.
 */

public abstract class AutonomousBaseV2 extends RoverHardwareV2 {
    protected void resetEncoders() {
        rfDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lfDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rbDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lbDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while (opModeIsActive() && rfDrive.getCurrentPosition() > 3 && lfDrive.getCurrentPosition() > 3) {
        }
        rfDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lfDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rbDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lbDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (opModeIsActive() && rfDrive.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER && lfDrive.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
        }
    }

    protected void resetArmEncoders() {
        collector.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while (opModeIsActive() && armRight.getCurrentPosition() > 3 && armLeft.getCurrentPosition() > 3) {
        }
        collector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (opModeIsActive() && armRight.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER && armLeft.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
        }
    }

    protected void waitSec(double seconds) {
        ElapsedTime t = new ElapsedTime(System.nanoTime());
        while (opModeIsActive() && t.time() <= seconds) {

        }
    }

    protected int getRightAbs() {
        return Math.abs(rfDrive.getCurrentPosition());
    }

    protected int getleftAbs() {
        return Math.abs(lfDrive.getCurrentPosition());
    }

    ///////////////////////////////////
    /////////////Movement//////////////
    ///////////////////////////////////

    protected void drive(double power) {
        setDrivePower(power, power, power, power);
    }

    protected void drive(double power, double inches) { // Moves the robot in a straight line at a determined power and distance
        resetEncoders();
        drive(power);
        double targetPosition = inches * ORBITAL20_PPR * DRIVE_GEAR_RATIO / WHEEL_CIRC;
        double starting = getAngle();
        double right = power;
        double left = power;
        while (opModeIsActive() && getleftAbs() <= targetPosition && getRightAbs() <= targetPosition) {
            double delta = starting - getAngle();
            right = power + (delta / 40);
            left = power - (delta / 40);

            if (Math.abs(right) > 1 || Math.abs(left) > 1) {
                right /= Math.max(right, left);
                left /= Math.max(right, left);
            }
            setDrivePower(right, left, right, left);
        }
        stopDrive();
        telemetry.addLine("Drove " + inches + " inches to target");
        telemetry.update();
    }

    protected void strafeRot(double power, double rotations) {
        resetEncoders();
        setDrivePower(-power, power, power, -power);
        //drive(power);
        double targetPosition = rotations * ORBITAL20_PPR * DRIVE_GEAR_RATIO;

        while (opModeIsActive() && getleftAbs() <= targetPosition && getRightAbs() <= targetPosition) {

        }
        stopDrive();
        telemetry.addLine("Strafed " + rotations + " Rotations to target");
        telemetry.update();
    }

    protected void turnHeading(double power, int degreeTarget) { //turns the robot at a set speed to a given z-axis value
        //resetAngle();
        heading = getAngle();

        //turn until gyro value is met
        while (opModeIsActive() && Math.abs(heading - degreeTarget) > 0) {
            if (heading > degreeTarget) {
                setDrivePower(-power, power, -power, power);
            }
            if (heading < degreeTarget) {
                setDrivePower(power, -power, power, -power);
            }
            heading = getAngle();
        }
        stopDrive();
        telemetry.addLine("Turned " + degreeTarget + "degrees to target");
        telemetry.update();
        resetAngle();
    }

    ///////////////////////////////////
    ///////////////Tasks///////////////
    ///////////////////////////////////

    protected void sample(){
        waitSec(1);
        if (gold_position == GOLD_POSITION.RIGHT){
            drive(-0.3, 12);
            strafeRot(-1, 4);
            strafeRot(1, 4);
            turnHeading(0.3, 90);

        } else if (gold_position == GOLD_POSITION.CENTER){
            strafeRot(-1, 4);
            strafeRot(1,4 );
            turnHeading(0.3, 90);

        } else {
            drive(0.3, 12);
            strafeRot(-1, 4);
            strafeRot(1, 4);
            turnHeading(0.3, 90);

        }
    }

    protected void land(){
        rotArm(2, -1);
        hangLock.setPosition(0.5);
        waitSec(0.5);
        rotArm(60,0.5);
        strafeRot(0.5, 3);
    }

    ///////////////////////////////////
    /////////////////Arm///////////////
    ///////////////////////////////////

    protected void rotArm(int degrees, double power){

        int target = NEVEREST60_PPR * ARM_RATIO * degrees / 360;

        while (Math.abs(armRight.getCurrentPosition()) < target ){
            armLeft.setPower(power);
            armRight.setPower(power);
        }
        armRight.setPower(0);
        armLeft.setPower(0);

        telemetry.addLine("Rotated Arm " + degrees + " degrees");
        telemetry.update();

    }

    protected void extendArm(int inches, double power){

        int target =  (int) (NEVEREST40_PPR * inches / Math.PI);

        while (Math.abs(armExtension.getCurrentPosition()) < target && !isStopRequested()){
            armExtension.setPower(power);
        }
        armExtension.setPower(0);

        telemetry.addLine("Extended Arm " + inches + " inches");
        telemetry.update();

    }

    protected void collectorTime(int seconds, double power){

        collector.setPower(power);
        waitSec(seconds);
        collector.setPower(0);
    }

    ///////////////////////////////////
    ///////////////Gyro////////////////
    ///////////////////////////////////

    protected void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    private int getAngle() {

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return (int) angles.firstAngle;
    }

    /////////////////////////////////////////////////
    ////////////////////Tensor Flow//////////////////
    /////////////////////////////////////////////////

    protected void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = "ATMaWPX/////AAABmZSp02GUa07oranA3Pd4lFdFBNnwHNZGGVH5c4S2XFLBGoC8s3a5qi5VEb6If/Xx/hl6YMfe0BbeThv0ZoAiC7i2A/AuHEtqsNdpx5loSt5uV4DGnw860ZPto6y7NN8cpjr+3rhDwriTQXGgoJ5fPSvI/QhfXtZTz0peh533l76mxJ4lKLNqHWzYZiG5CptqisPRrVQl+fIv2AjOg9vhNxZMEq9yT3KQNVxK88vriPIaOzDeN8Qy8WeQIbOS5tEP88Ax/tEwsA4DTHr80+6ngkdsC4qXZNkS/ooy9VLTev55fjqxhlyLZm5/Xs+svNFMwlV/0Shn3ssiAxFuffDymF24wLPmfaB/1G2GBT4VxISO";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    protected void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }


    protected void getGoldPos() {

        if (tfod != null) {
            tfod.activate();
        }

        while(gold_position == null && !isStopRequested()) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                if (updatedRecognitions.size() >= 2) {
                    int goldMineralX = -1;
                    int silverMineral1X = -1;
                    int silverMineral2X = -1;

                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            goldMineralX = (int) recognition.getLeft();
                        } else if (silverMineral1X == -1) {
                            silverMineral1X = (int) recognition.getLeft();
                        } else {
                            silverMineral2X = (int) recognition.getLeft();
                        }
                    }

                    if (goldMineralX == -1) {
                        gold_position = GOLD_POSITION.RIGHT;
                    } else if (goldMineralX > silverMineral1X) {
                        gold_position = GOLD_POSITION.CENTER;
                    } else {
                        gold_position = GOLD_POSITION.LEFT;
                    }

                    if (gold_position == GOLD_POSITION.LEFT) {
                        telemetry.addData("Position", "Left");
                    } else if (gold_position == GOLD_POSITION.CENTER) {
                        telemetry.addData("Position", "Center");
                    } else {
                        telemetry.addData("Position", "Right");
                    }

                    telemetry.update();



                }
            }
        }

        tfod.deactivate();
    }
}