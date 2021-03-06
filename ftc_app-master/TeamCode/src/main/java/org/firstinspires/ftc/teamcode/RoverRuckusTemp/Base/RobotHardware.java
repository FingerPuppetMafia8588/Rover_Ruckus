package org.firstinspires.ftc.teamcode.RoverRuckusTemp.Base;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
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

    protected DcMotor hang;

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

    //final variables for moving robot to distance
    protected final double WHEEL_DIAMTER = 4;
    protected final double WHEEL_CIRC = WHEEL_DIAMTER*Math.PI;
    protected final double ORBITAL20_PPR = 537.6;
    protected final double DRIVE_GEAR_RATIO = 1;

    protected final double FORWARD_RATIO = 1;
    protected final double STRAFE_RATIO = 1;
    protected final double TURN_RATIO = 0.7;

    protected final double GOLD_RATIO = 2.6;

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

        liftL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftR.setDirection(DcMotorSimple.Direction.REVERSE);

        hang = hardwareMap.dcMotor.get("hang");

        hang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hang.setDirection(DcMotorSimple.Direction.REVERSE);

        dumpL = hardwareMap.servo.get("dump_left");
        dumpL.setPosition(1);

        collectorL = hardwareMap.servo.get("collectorL");

        collectorR = hardwareMap.servo.get("collectorR");

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

            collectorL.setPosition(.5);
            collectorR.setPosition(0.5);

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

    protected void MecanumFormula(double forward, double strafe, double turning){

        double rfPower, rbPower, lfPower, lbPower;
        double max;

        //reset powers
        lfPower = 0;
        lbPower = 0;
        rfPower = 0;
        rbPower = 0;

        //handle forward/backward movement
        lfPower += forward;
        lbPower += forward;
        rfPower += forward;
        rbPower += forward;

        //handle strafing movement
        lfPower += strafe;
        lbPower -= strafe;
        rfPower -= strafe;
        rbPower += strafe;

        //handle turning movement
        lfPower += turning;
        lbPower += turning;
        rfPower -= turning;
        rbPower -= turning;

        //scale powers of exceeding those of the motor
        max = Math.abs(lfPower);
        if (Math.abs(lbPower) > max){
            max = Math.abs(lbPower);
        }
        if (Math.abs(rfPower) > max){
            max = Math.abs(rfPower);
        }
        if (Math.abs(rbPower) > max){
            max = Math.abs(rbPower);
        }

        if (max > 1){
            lbPower /= max;
            lfPower /= max;
            rbPower /= max;
            rfPower /= max;
        }

        setDrivePower(rfPower, lfPower, rbPower, lbPower);
    }

    protected void FieldCentricDrive(){
        double x1 = gamepad1.left_stick_x, y1 = -gamepad1.left_stick_y;
        double v = Math.sqrt(x1 * x1 + y1 * y1);
        double theta = Math.atan2(x1, y1);
        double current = Math.toRadians(getGlobal() % 360);
        drive(theta + current, v, gamepad1.right_stick_x);
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

    protected Wheels getWheels(double direction, double velocity, double rotationVelocity) {
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

    protected static double ma(double... xs) {
        double ret = 0.0;
        for (double x : xs) {
            ret = Math.max(ret, Math.abs(x));
        }
        return ret;
    }

    protected void drive(double direction, double velocity, double rotationVelocity) {
        Wheels w = getWheels(direction, velocity, rotationVelocity);
        lfDrive.setPower(w.lf);
        rfDrive.setPower(w.rf);
        lbDrive.setPower(w.lr);
        rbDrive.setPower(w.rr);
    }

    protected double getGlobal(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }
}


