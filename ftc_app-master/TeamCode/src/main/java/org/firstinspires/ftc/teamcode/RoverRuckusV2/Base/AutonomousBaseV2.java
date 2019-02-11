package org.firstinspires.ftc.teamcode.RoverRuckusV2.Base;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
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
        setDrivePower(-power, power, power, -power);
        double targetPosition = rotations * ORBITAL20_PPR * DRIVE_GEAR_RATIO;

        while (opModeIsActive() && getleftAbs() <= targetPosition && getRightAbs() <= targetPosition) {

        }
        stopDrive();
        telemetry.addLine("Strafed " + rotations + " Rotations to target");
        telemetry.update();
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
        double retStrafe;
        if (autoType == AutoType.CRATER){
            retStrafe = 1.0;
        } else {
            retStrafe = 1.5;
        }

        waitSec(0.5);
        //drive to needed position
        drive(0.5, 7.5);
        turnHeading(0.25, 90);
        turnHeading(0.15, 90);
        drive(-0.3, 3.9);
        waitSec(0.2);
        correctPositon();
        waitSec(1);
        //scan minerals
        getGoldPos();
        waitSec(0.3);

        if (gold_position == GOLD_POSITION.RIGHT){ //knock off right mineral
            drive(-0.3, 12);
            waitSec(0.5);
            strafeRot(0.5, 1.5);
            strafeRot(-0.5, retStrafe);
            turnHeading(0.3, 90);
            turnHeading(0.2, 90);
            waitSec(0.3);
            drive(0.3, 29);


        } else if (gold_position == GOLD_POSITION.CENTER){ //knock off center mineral
            drive(0.3, 3);
            waitSec(0.5);
            strafeRot(0.5, 1.5);
            strafeRot(-0.5, retStrafe);
            turnHeading(0.3, 90);
            turnHeading(0.2, 90);
            waitSec(0.3);
            drive(0.3, 14);

        } else { // knock off left mineral
            drive(0.3, 17);
            waitSec(0.5);
            strafeRot(0.5, 1.5);
            strafeRot(-0.5, retStrafe);
            turnHeading(0.3, 90);
            turnHeading(0.2, 90);

        }
    }

    /**
     * lands the robot during autonomous
     */
    protected void land(){
        waitSec(0.3);
        rotArm(2, -1);
        hangLock.setPosition(1);
        waitSec(2);
        armRight.setPower(0.5);
        armLeft.setPower(0.5);
        while(Math.abs(armRight.getCurrentPosition()) < 2){

        }
        armRight.setPower(0);
        armLeft.setPower(0);

        rotArm(5, 1);

        ElapsedTime t = new ElapsedTime();
        drive(-0.2);
        while (t.time() <= 1){

        }
        drive(0);
        turnHeading(0.3, 10);
        strafeRot(1, 0.4);
        rotArm(75, -1);
        turnHeading(0.3, 0);
        strafeRot(-0.4, 0.23);
        waitSec(0.2);
        alignLander(-0.3, 3);
        waitSec(0.3);
        turnHeading(0.35, 0);
        turnHeading(0.15, 0);
        armLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * used to align against the lander in autonomous with a time kill-switch in case the robot locks up
     * @param power speed to move against the lander
     * @param inches distance to move
     */
    protected void alignLander(double power, int inches){
        resetEncoders();
        ElapsedTime t = new ElapsedTime(System.nanoTime());
        drive(power);
        double targetPosition = inches * ORBITAL20_PPR * DRIVE_GEAR_RATIO / WHEEL_CIRC;
        double starting = getAngle();
        double right = power;
        double left = power;
        while (opModeIsActive() && getleftAbs() <= targetPosition && getRightAbs() <= targetPosition && t.time() < 2) {
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

    ///////////////////////////////////
    /////////////////Arm///////////////
    ///////////////////////////////////

    /**
     * moves the arm a set amount of degrees at a given speed
     * @param degrees amount to move the arm
     * @param power speed/direction to move the arm
     */
    protected void rotArm(int degrees, double power){

        resetArmEncoders();
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

        while (Math.abs(armExtension.getCurrentPosition()) < target && !isStopRequested()){
            armExtension.setPower(power);
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
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * initializes the parameters for the use of tensor flow scanning
     */
    protected void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    /**
     * scans the minerals and applies the position to a global variable
     */
    protected void getGoldPos() {

        if (tfod != null) {
            tfod.activate();
        }
        ElapsedTime t = new ElapsedTime(System.nanoTime());
        while(gold_position == null && opModeIsActive()) {
            if (t.time() >= 4){
                gold_position = GOLD_POSITION.CENTER;
            }
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                if (updatedRecognitions.size() >= 1) {
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

    /**
     * uses the position of known minerals to correct the robots position for scanning
     */
    public void correctPositon(){
        if (tfod != null) {
            tfod.activate();
        }
        ElapsedTime t = new ElapsedTime(System.nanoTime());
        boolean hasTwo = false;
        int counter = 0;
        while (!hasTwo && opModeIsActive()){
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                int goldMineralX = -1;
                int silverMineral1X = -1;
                int silverMineral2X = -1;
                if (updatedRecognitions.size() >= 1){

                    counter++;
                    if (counter == 1){t.reset();}
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            goldMineralX = (int) recognition.getLeft();
                        } else if (silverMineral1X == -1) {
                            silverMineral1X = (int) recognition.getLeft();
                        } else {
                            silverMineral2X = (int) recognition.getLeft();
                        }
                    }
                }
                if (updatedRecognitions.size() == 1){
                    int mineralX = -1;
                    if (goldMineralX > 0){
                        mineralX = goldMineralX;
                    } else if (silverMineral1X > 0){
                        mineralX = silverMineral1X;
                    }

                    if (mineralX < 600 && mineralX > 0){
                        drive(-0.15);
                    } else {
                        drive(0.15);
                    }
                } else if (updatedRecognitions.size() == 2 || t.time() >= 5){
                    drive(0);
                    hasTwo = true;
                }
            }
        }
    }
}
