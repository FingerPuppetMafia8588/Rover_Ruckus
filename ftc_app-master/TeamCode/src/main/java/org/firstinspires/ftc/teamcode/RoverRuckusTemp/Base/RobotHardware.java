package org.firstinspires.ftc.teamcode.RoverRuckusTemp.Base;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

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

    protected DcMotor liftR;
    protected DcMotor liftL;

    protected Servo dumpL;

    protected Servo collectorL;
    protected Servo collectorR;

    protected Servo armL;
    protected Servo armR;

    protected Servo marker;

    protected ColorSensor colorL;
    protected ColorSensor colorR;




    // declares gyro
    protected BNO055IMU imu;
    protected Orientation lastAngles = new Orientation();
    protected Orientation angles;

    protected double globalAngle;

    protected int heading;


    protected final double WHEEL_DIAMTER = 4;
    protected final double WHEEL_CIRC = WHEEL_DIAMTER*Math.PI;
    protected final double ORBITAL20_PPR = 537.6;
    protected final double DRIVE_GEAR_RATIO = 1;

    protected final double FORWARD_RATIO = 1;
    protected final double STRAFE_RATIO = 1;
    protected final double TURN_RATIO = 0.7;

    @Override
    protected void initRobot(RobotRunType robotRunType){

        rfDrive = hardwareMap.dcMotor.get("right_front");
        lfDrive = hardwareMap.dcMotor.get("left_front");
        rbDrive = hardwareMap.dcMotor.get("right_back");
        lbDrive = hardwareMap.dcMotor.get("left_back");

        lfDrive.setDirection(DcMotor.Direction.REVERSE);
        lbDrive.setDirection(DcMotor.Direction.REVERSE);

        rfDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lfDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rbDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lbDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rfDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lfDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rbDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lbDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        liftL = hardwareMap.dcMotor.get("lift_left");
        liftR = hardwareMap.dcMotor.get("lift_right");

        liftR.setDirection(DcMotorSimple.Direction.REVERSE);

        dumpL = hardwareMap.servo.get("dump_left");
        dumpL.setPosition(1);

        collectorL = hardwareMap.servo.get("collectorL");
        collectorL.setPosition(.5);

        collectorR = hardwareMap.servo.get("collectorR");
        collectorR.setPosition(0.5);

        armL = hardwareMap.servo.get("armL");
        armL.setPosition(0);
        armR = hardwareMap.servo.get("armR");
        armR.setPosition(1);

        marker = hardwareMap.servo.get("marker");
        marker.setPosition(0);

        colorL = hardwareMap.get(ColorSensor.class, "colorL");
        colorR = hardwareMap.get(ColorSensor.class, "colorR");



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

    protected static class Wheels {
        public double lf, lr, rf, rr;

        public Wheels(double lf, double rf, double lr, double rr) {
            this.lf = lf;
            this.rf = rf;
            this.lr = lr;
            this.rr = rr;
        }
    }
}
