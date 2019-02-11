package org.firstinspires.ftc.teamcode.RoverRuckusV2.Tests;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.RoverRuckusTemp.Base.RobotRunType;
import org.firstinspires.ftc.teamcode.RoverRuckusV2.Base.AutonomousBaseV2;

import java.util.List;

/**
 * Created by isaac.blandin on 2/6/19.
 */

public class CorrectingScanTest extends AutonomousBaseV2{

    @Override
    public void runOpMode(){

        initRobotV2(RobotRunType.AUTONOMOUS);



    }

    public void ScanningTest(){
        if (tfod != null) {
            tfod.activate();
        }

        while(gold_position == null && opModeIsActive()) {
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



                } else if (updatedRecognitions.size() == 1){

                } else {

                }
            }
        }

        tfod.deactivate();
    }

    public void adjustPosition(int mineralX){
    }

}
