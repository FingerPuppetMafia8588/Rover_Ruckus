package org.firstinspires.ftc.teamcode.RoverRuckusV2.Base;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.RoverRuckusTemp.Base.RobotRunType;

/**
 * Created by isaac.blandin on 1/7/19.
 */

public abstract class RoverHardwareV2 extends RobotBaseV2 {

    //declares drive motors
    protected DcMotor rfDrive;
    protected DcMotor lfDrive;
    protected DcMotor rbDrive;
    protected DcMotor lbDrive;
    //declares arm rotation motors
    protected DcMotor armRight;
    protected DcMotor armLeft;
    //declares arm extension motor
    protected DcMotor armExtension;
    //declares collector motor
    protected DcMotor collector;

    //declares servo to lock before landing
    protected Servo hangLock;
    //declares mineral dumping servos
    protected Servo dump;

    // declares gyro and gyro variables
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

    protected final int NEVEREST60_PPR = 1680;
    protected final int NEVEREST40_PPR = 1120;
    protected final int ARM_RATIO = 9;

    protected final double TURN_RATIO = 0.7;

    //declares variables and parameters for tensor flow scanning
    protected static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    protected static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    protected static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    protected static final String VUFORIA_KEY = "ATMaWPX/////AAABmZSp02GUa07oranA3Pd4lFdFBNnwHNZGGVH5c4S2XFLBGoC8s3a5qi5VEb6If/Xx/hl6YMfe0BbeThv0ZoAiC7i2A/AuHEtqsNdpx5loSt5uV4DGnw860ZPto6y7NN8cpjr+3rhDwriTQXGgoJ5fPSvI/QhfXtZTz0peh533l76mxJ4lKLNqHWzYZiG5CptqisPRrVQl+fIv2AjOg9vhNxZMEq9yT3KQNVxK88vriPIaOzDeN8Qy8WeQIbOS5tEP88Ax/tEwsA4DTHr80+6ngkdsC4qXZNkS/ooy9VLTev55fjqxhlyLZm5/Xs+svNFMwlV/0Shn3ssiAxFuffDymF24wLPmfaB/1G2GBT4VxISO";
    protected VuforiaLocalizer vuforia;
    protected TFObjectDetector tfod;
    public GOLD_POSITION gold_position;

    /**
     * sets all of the initialization values of the robot
     *
     * @param robotRunType tells whether the opmode is TeleOp or Autonomous in order to change needed
     *                     functions such as servo positions
     */
    protected void initRobotV2 (RobotRunType robotRunType){


        // set up drive motors
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

        // set up arm rotation motors
        armRight = hardwareMap.dcMotor.get("arm_right");
        armLeft = hardwareMap.dcMotor.get("arm_left");

        armRight.setDirection(DcMotor.Direction.REVERSE);

        armRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        armRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // set up extension motor
        armExtension = hardwareMap.dcMotor.get("arm_extension");

        armExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        armExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //set up collector motor
        collector = hardwareMap.dcMotor.get("collector");

        collector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        collector.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //set up hanging lock servo
        hangLock = hardwareMap.servo.get("hang_lock");

        //set up dumping servo
        dump = hardwareMap.servo.get("dump");

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

        if (robotRunType == RobotRunType.AUTONOMOUS){

            //set autonomous servo positions
            hangLock.setPosition(0);
            //dump.setPosition(0.08);

        }
    }

    /**
     * Method to apply power to all four of the motors with a single line of code
     *
     * @param rightFrontPower power of the right front wheel
     * @param leftFrontPower power of the left front wheel
     * @param rightBackPower power of the right back wheel
     * @param leftBackPower power of the left back wheel
     */
    protected void setDrivePower (double rightFrontPower, double leftFrontPower, double rightBackPower, double leftBackPower) {
        rfDrive.setPower(rightFrontPower);
        lfDrive.setPower(leftFrontPower);
        rbDrive.setPower(rightBackPower);
        lbDrive.setPower(leftBackPower);

    }

    /**
     * stops all of the drive wheels of the robot
     */
    protected void stopDrive() {
        setDrivePower(0, 0, 0 ,0);
    }


    /**
     * used to drive a holonomic drive train in a conventional method using three variables
     * now obsolete and replaced with Field Centric Drive
     * @param forward power applied for forward-backward movement
     * @param strafe power applied for lateral movement
     * @param turning power applied for turning
     */
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

    /**
     * method to drive the robot using field-centric drive
     *
     * movement - left joystick
     * rotation - right joystick x-axis
     * slow-mode - right bumper
     */
    protected void FieldCentricDrive(){

        double turnRatio = 0.7;
        if (gamepad1.right_bumper){
            turnRatio = 0.3;
        }

        //pull coordinate values from x and y axis of joystick
        double x1 = gamepad1.left_stick_x, y1 = -gamepad1.left_stick_y;
        //create polar vector of given the joystick values
        double v = Math.sqrt(x1 * x1 + y1 * y1);
        double theta = Math.atan2(x1, y1);
        //get radian value of the robots angle
        double current = Math.toRadians(getGlobal() % 360);
        //apply values to vector-based holonomic drive
        drive(theta + current, v, gamepad1.right_stick_x * turnRatio);
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

    /**
     * method used to drive a holonomic drive train given a vector for direction and power
     *
     * @param direction angular vector in radians which gives the direction to move
     * @param velocity speed in which the robot should be moving
     * @param rotationVelocity speed which the robot should be rotating
     * @return Wheels - creates new Wheels class
     */
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

    /**
     * method to scale values to within a certain range
     *
     * @param xs values to be used to create the scale
     * @return double - scale to be used by the upper shell
     */
    protected static double ma(double... xs) {
        double ret = 0.0;
        for (double x : xs) {
            ret = Math.max(ret, Math.abs(x));
        }

        return ret;
    }

    /**
     * Uses the wheels classes to apply the vector driven holonomic formula to the four wheels of a
     * holonomic drivetrain
     *
     * @param direction angular vector in radians which gives the direction to move
     * @param velocity speed in which the robot should be moving
     * @param rotationVelocity speed which the robot should be rotating
     */
    protected void drive(double direction, double velocity, double rotationVelocity) {
        RoverHardwareV2.Wheels w = getWheels(direction, velocity, rotationVelocity);
        lfDrive.setPower(w.lf);
        rfDrive.setPower(w.rf);
        lbDrive.setPower(w.lr);
        rbDrive.setPower(w.rr);
    }

    /**
     * method which allows the rev integrated gyroscope to be used without the wrap around values
     * where it resets at 180 degrees
     *
     * @return double - unrestricted gyroscope value of the robot
     */
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

    protected enum GOLD_POSITION {
        LEFT, RIGHT, CENTER
    }
}
