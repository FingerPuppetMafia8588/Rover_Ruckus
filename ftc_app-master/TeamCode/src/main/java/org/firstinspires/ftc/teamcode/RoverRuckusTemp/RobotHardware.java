package org.firstinspires.ftc.teamcode.RoverRuckusTemp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by isaac.blandin on 8/28/18.
 */

public abstract class RobotHardware extends RobotBase {

    //declares drive motors
    protected DcMotor rfDrive;
    protected DcMotor lfDrive;
    protected DcMotor rbDrive;
    protected DcMotor lbDrive;

    // declares gyro
    protected BNO055IMU imu;
    protected Orientation lastAngles = new Orientation();
    protected Orientation angles;

    double globalAngle;

    protected int heading;


    protected final double WHEEL_DIAMTER = 4;
    protected final double WHEEL_CIRC = WHEEL_DIAMTER*Math.PI;
    protected final double ORBITAL20_PPR = 537.6;
    protected final double DRIVE_GEAR_RATIO = 1;

    @Override
    protected void initRobot(RobotRunType robotRunType){

        rfDrive = hardwareMap.dcMotor.get("right_front");
        lfDrive = hardwareMap.dcMotor.get("left_front");
        rbDrive = hardwareMap.dcMotor.get("right_back");
        lbDrive = hardwareMap.dcMotor.get("left_back");

        lfDrive.setDirection(DcMotor.Direction.REVERSE);
        lbDrive.setDirection(DcMotor.Direction.REVERSE);

        rfDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lfDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rbDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lbDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // initialize gyro if starting in autonomous
        if (robotRunType == RobotRunType.AUTONOMOUS){
            //initialize gyro
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

            parameters.mode = BNO055IMU.SensorMode.IMU;
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.loggingEnabled = false;

            imu = hardwareMap.get(BNO055IMU.class, "imu");

            imu.initialize(parameters);

            //post to telemetry when gyro is calibrating
            telemetry.addData("Mode", "Calibrating");
            telemetry.update();

            //post to telemetry when gyro is calibrated
            while (!isStopRequested() && !imu.isGyroCalibrated()){
                sleep(50);
                idle();
            }

            telemetry.addData("Mode", "waiting for start");
            telemetry.addData("imu calibration", imu.getCalibrationStatus().toString());
            telemetry.update();
        }

    }
    // method to simplify setting drive power for the robot drive motors
    protected void setDrivePower (double rightFrontPower, double leftFrontPower, double rightBackPower, double leftBackPower) {
        rfDrive.setPower(rightFrontPower);
        lfDrive.setPower(leftFrontPower);
        rbDrive.setPower(rightBackPower);
        lbDrive.setPower(leftBackPower);

    }

    //method to easily stop the robot
    protected void stopDrive() {
        setDrivePower(0, 0, 0 ,0);
    }

}
