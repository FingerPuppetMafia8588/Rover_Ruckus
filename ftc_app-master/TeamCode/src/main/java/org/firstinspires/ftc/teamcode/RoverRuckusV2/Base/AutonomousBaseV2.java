package org.firstinspires.ftc.teamcode.RoverRuckusV2.Base;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import java.util.List;

/**
 * Created by isaac.blandin on 1/16/19.
 */

public abstract class AutonomousBaseV2 extends RoverHardwareV2 {
    /**
     * resets the drive motor encoders
     */
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
            //waiting
        }
    }

    /**
     * resets the encoders in the main arm
     */
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

    /**
     * pauses the code for a set amount of seconds
     * @param seconds time to wait before resuming code
     */
    protected void waitSec(double seconds) {
        ElapsedTime t = new ElapsedTime(System.nanoTime());
        while (opModeIsActive() && t.time() <= seconds) {

        }
    }

    /**
     * gets the absolute value of the right drive motor encoder positions
     * @return int - absolute value of right front wheel
     */
    protected int getRightAbs() {
        return Math.abs(rfDrive.getCurrentPosition());
    }

    /**
     * gets the absolute value of the left drive motor encoder positions
     * @return int - absolute value of left front wheel
     */
    protected int getleftAbs() {
        return Math.abs(lfDrive.getCurrentPosition());
    }

    ///////////////////////////////////
    /////////////Movement//////////////
    ///////////////////////////////////

    /**
     * sets all of the drive motors to the same value
     * @param power power in which the drive motors will be set to
     */
    protected void drive(double power) {
        setDrivePower(power, power, power, power);
    }

    /**
     * drives the robot in a straight line for a given distance
     * @param power speed which the robot should move at
     * @param inches target for which the robot should try to stop at
     */
    protected void drive(double power, double inches) {
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

    /**
     * moves the robot straight laterally at a given power and distance
     * @param power speed at which it should move
     * @param rotations number of rotations the wheels should spin
     */
    protected void strafeRot(double power, double rotations) {
        resetEncoders();
        waitSec(0.2);
        setDrivePower(-power, power, power, -power);
        double targetPosition = rotations * ORBITAL20_PPR * DRIVE_GEAR_RATIO;

        while (Math.abs((double)rfDrive.getCurrentPosition()) <= targetPosition) {

        }
        stopDrive();
        telemetry.addLine("Strafed " + rotations + " Rotations to target");
        telemetry.update();
    }
    protected void strafeTime(double power, double rotations) {
        resetEncoders();
        setDrivePower(-power, power, power, -power);
        double targetPosition = rotations * ORBITAL20_PPR * DRIVE_GEAR_RATIO;

        waitSec(rotations);
        stopDrive();
        telemetry.addLine("Strafed " + rotations + " Rotations to target");
        telemetry.update();
    }

    protected void driveVector(double target, double direction, double power, double rot, int heading, boolean time){
        resetEncoders();
        direction = Math.toRadians(direction);
        double adjustment;
        if (rot == 0){
            adjustment = power/2;
        } else {
            adjustment = power;
        }
        if (power < 0){
            adjustment = -adjustment;
        }
        target = target * ORBITAL20_PPR * DRIVE_GEAR_RATIO / WHEEL_CIRC;
        drive(direction, power, 0);
        while(getRightAbs() < target && getleftAbs() < target && opModeIsActive()){
            if (Math.abs(getAngle() - heading) > 3)
                if (getAngle() < heading){
                    drive(direction, power, -adjustment);
                } else if (getAngle() > heading){
                    drive(direction, power, adjustment);
                } else {
                    drive(direction, power, 0);
                }
        }
        stopDrive();
    }

    public double getVelocity(){
        return imu.getVelocity().xVeloc;
    }
    /**
     * turns the robot at a given speed to target angle
     * @param power speed at which to turn
     * @param degreeTarget target angle which it will try to turn to
     */
    protected void turnHeading(double power, int degreeTarget) {
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

    /**
     * scans and knocks off the gold mineral in autonomous
     */
    protected void sample(AutoType autoType){

        turnHeading(0.23, 13);
        turnHeading(0.19, 13);
        waitSec(0.2);
        //scan minerals
        getGoldPos();
        waitSec(0.2);
        turnHeading(0.2, 0);
        waitSec(0.2);
        turnHeading(0.2, 0);
        //drive(0.3, 16);
        waitSec(0.2);

        if (autoType == AutoType.DEPOT) {
            if (gold_position == GOLD_POSITION.RIGHT) { //knock off right mineral

                driveVector(43, 53, 0.5, 0, 0, false);
                driveVector(6, 0, 0.5, 0, 0, false);
                driveVector(40, -90, 0.5, 0.25, -45, false);
                collectorTime(1, -1);
                turnHeading(0.2, -47);
                drive(-0.4, 65);


            } else if (gold_position == GOLD_POSITION.CENTER) { //knock off center mineral

                driveVector(40, 0, 0.3, 0, 0, false);
                driveVector(10, -90, 0.5, 0.25, -45, false);
                collectorTime(1, -1);
                turnHeading(0.2, -47);
                drive(-0.4, 60);


            } else { // knock off left mineral

                driveVector(2, -90, 0.3, 0, 0, false);
                driveVector(45, -55, 0.5, 0, 0, false);
                driveVector(16, -90, 0.5, 0.35, -45, false);
                turnHeading(0.2, -45);
                drive(0.4, 24);
                collectorTime(1, -1);
                turnHeading(0.2, -47);
                drive(-0.4, 56);
            }
        } else if (autoType == AutoType.CRATEREARLY){
            if (gold_position == GOLD_POSITION.RIGHT){

                driveVector(43, 53, 0.5, 0, 0, false);

            } else if (gold_position == GOLD_POSITION.CENTER){

                driveVector(40, 0, 0.3, 0, 0, false);

            } else {

                driveVector(2, -90, 0.3, 0, 0, false);
                driveVector(45, -55, 0.5, 0, 0, false);

            }
        }
    }

    protected void craterClaim (){
        driveVector(19, 45, 0.5, 0.53, 90, false);
        turnHeading(0.2, 88);
        drive(0.4, 29);
        driveVector(15, 45, 0.5, 0.5, 135, false);
        waitSec(0.1);
        turnHeading(0.2, 135);
        drive(0.5, 42);
        collectorTime(1, -1);
        //turnHeading(0.2, 135);
        drive(-0.5, 27);
        turnHeading(0.2, 90);
        drive(-0.4, 22);

    }

    protected void craterSampleEnd(){
        if (gold_position == GOLD_POSITION.RIGHT){
            drive(-0.4, 32.5);
        } else if (gold_position == GOLD_POSITION.CENTER){
            drive(-0.4, 15);
        } else {

        }

        turnHeading(0.3, 0);
        drive(0.3, 12);
    }

    /**
     * lands the robot during autonomous
     */
    protected void land(){
        waitSec(0.3);
        rotArm(10, -1);
        waitSec(0.2);
        hangLock.setPosition(1);
        waitSec(0.5);
        hangLock.setPosition(0);
        waitSec(0.5);
        hangLock.setPosition(1);
        waitSec(0.3);
        rotArm(72, 1);
        driveVector(1, 0, 0.2, 0, 0, false);
        driveVector(2, 90, 0.3, 0, 0 , false);
        turnHeading(0.6, 17);
        turnHeading(0.3, 0);
        //turnHeading(0.2, 0);
        resetArmEncoders();
        rotArm(55, -1);



    }

    public void depotClaim(){
        extendArm(10, 1);
        resetArmEncoders();
        rotArm(20, -0.5);
        waitSec(0.2);
        collectorTime(1, -1);
        resetArmEncoders();
        rotArm(50, 1);
        waitSec(0.2);
        extendArm(9, -1);
    }

    ///////////////////////////////////
    /////////////////Arm///////////////
    ///////////////////////////////////

    /**
     * moves the arm a set amount of degrees at a given speed
     * @param degrees amount to move the arm
     * @param power speed/direction to move the arm
     */
    protected void rotArm(int degrees, double power){

        int target = NEVEREST60_PPR * ARM_RATIO * degrees / 360;

        while (Math.abs(armRight.getCurrentPosition()) < target && opModeIsActive()){
            armLeft.setPower(power);
            armRight.setPower(power);
        }
        armRight.setPower(0);
        armLeft.setPower(0);

        telemetry.addLine("Rotated Arm " + degrees + " degrees");
        telemetry.update();

    }

    /**
     * extends/retracts the arm a given distance at a given speed
     * @param inches distance to extend the arm out
     * @param power speed to move the arm out at
     */
    protected void extendArm(int inches, double power){

        int target =  (int) (NEVEREST40_PPR * inches / Math.PI);

        armExtension.setPower(power);
        while (Math.abs(armExtension.getCurrentPosition()) < target && !isStopRequested()){

        }
        armExtension.setPower(0);

        telemetry.addLine("Extended Arm " + inches + " inches");
        telemetry.update();

    }

    /**
     * runs the collector at a given speed for a duration of time
     * @param seconds time to run the collector for
     * @param power speed to set the collector to
     */
    protected void collectorTime(int seconds, double power){

        collector.setPower(power);
        waitSec(seconds);
        collector.setPower(0);
    }

    ///////////////////////////////////
    ///////////////Gyro////////////////
    ///////////////////////////////////

    /**
     * resets the angle of the gyroscope method
     */
    protected void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * gets the angle of the gyro
     * @return int - angular heading of the robot
     */
    private int getAngle() {

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return (int) angles.firstAngle;
    }

    /////////////////////////////////////////////////
    ////////////////////Tensor Flow//////////////////
    /////////////////////////////////////////////////

    /**
     * initializes the parameters for Vuforia detection
     */
    protected void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = "ATMaWPX/////AAABmZSp02GUa07oranA3Pd4lFdFBNnwHNZGGVH5c4S2XFLBGoC8s3a5qi5VEb6If/Xx/hl6YMfe0BbeThv0ZoAiC7i2A/AuHEtqsNdpx5loSt5uV4DGnw860ZPto6y7NN8cpjr+3rhDwriTQXGgoJ5fPSvI/QhfXtZTz0peh533l76mxJ4lKLNqHWzYZiG5CptqisPRrVQl+fIv2AjOg9vhNxZMEq9yT3KQNVxK88vriPIaOzDeN8Qy8WeQIbOS5tEP88Ax/tEwsA4DTHr80+6ngkdsC4qXZNkS/ooy9VLTev55fjqxhlyLZm5/Xs+svNFMwlV/0Shn3ssiAxFuffDymF24wLPmfaB/1G2GBT4VxISO";
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

    }

    /**
     * initializes the parameters for the use of tensor flow scanning
     */
    protected void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.34;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    /**
     * scans the minerals and assigns the position to a global variable
     */
    protected void getGoldPos() {

        if (tfod != null) {
            tfod.activate();
        }
        ElapsedTime t = new ElapsedTime(System.nanoTime());
        while(gold_position == null && opModeIsActive()) {
            if (t.time() >= 5){
                gold_position = GOLD_POSITION.CENTER;
            }
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
