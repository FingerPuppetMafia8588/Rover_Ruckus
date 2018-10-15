package org.firstinspires.ftc.teamcode.RoverRuckusTemp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by isaac.blandin on 5/21/18.
 */

@TeleOp(name = "gyroTest")
public class revGyroExample extends LinearOpMode {

    // declares gyro
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    Orientation angles;

    @Override
    public void runOpMode(){

        //initialize gyro
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
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

        waitForStart();
        while (opModeIsActive()){

            //pull angular orientation from gyro
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            //post gyro values to telemetry
            telemetry.addData("heading", angles.firstAngle);
            telemetry.addData("y-axis", angles.secondAngle);
            telemetry.addData("x-axis", angles.thirdAngle);

            telemetry.update();
        }
        stop();
    }
}
