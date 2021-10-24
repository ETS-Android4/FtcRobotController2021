// Fibonacci Team 14126 Autonomous Depot side code
// March 2nd, 2019 - Compiled for Non-Arm robot

// Setup to hit a middle gold mineral off

// Commented 01.17.2019 by Ben, Aubrie

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name="Depot", group="Autonomous1")
@Disabled
public class Depot extends LinearOpMode
{
    // Define Variables
    private DcMotor leftFrontMotor = null;
    private DcMotor leftBackMotor = null;
    private DcMotor rightFrontMotor = null;
    private DcMotor rightBackMotor = null;
    private CRServo servo0 = null;
    private Servo servo1 = null;
    double contPower, servo1_pos;
    private DcMotor boomMotor = null;
    DigitalChannel l_limit;
    DigitalChannel u_limit;
    int left_degrees=0;
    int right_degrees=0;
    int drivetime=0;
    double motorpower=0;
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "Ae6MFyH/////AAABmf06rZVcN0VqtDgoLd1KtAcowILYnLgT+SXkGwocAFEpSEmZiGI51vbPAL/QfAgSIhIpPrAK3Fl+vReEgcd1kU5az1pYTI01VqIV1+3ELPbCEcnNMdw3Rs7L8tMMsyfY2nebMlSFfgn6rkfJnnoQEnr4gCGHB1K/7VVZsFg7AlG4SPJ1bORRlhkFf0xIP28Tvr80YnC06hnsL2AQgOJtGrPv7HUO04hWxe9jLMdwHhmnBu/FZfovI9A6EjrzB72joGNTzuLA5R2bBGKW6AsrkKcgw1y50EFOqanZ19gWBX7bc1OExeaNeNIMaGzbMV8jwVahVndqS4EiLc9FuudY21tw4b4jupvhSYUiSGBMtLmh";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private boolean MissionComplete = false;
    private int goldPosition;

    // function for driving forward on degrees with all four motors
    // Paramaters:
    public void drivedeg (int left_degrees, int right_degrees, double motorpower, int drivetime){
        // tell the robot where to go
        leftFrontMotor.setTargetPosition(left_degrees);
        leftBackMotor.setTargetPosition(left_degrees);
        rightFrontMotor.setTargetPosition(right_degrees);
        rightBackMotor.setTargetPosition(right_degrees);
        // actually make the robot go there
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //telling the motors how fast to go
        leftFrontMotor.setPower(motorpower);
        leftBackMotor.setPower(motorpower);
        rightFrontMotor.setPower(motorpower);
        rightBackMotor.setPower(motorpower);
        //telling the robot how long to wait beforemoving on to the next line of code
        sleep(drivetime);
        // making the telemetry to update
        telemetry.addLine();
//        long startTime = System.currentTimeMillis(); //fetch starting time
//        //while (false || (System.currentTimeMillis() - startTime) < 5000) {
//        while (leftFrontMotor.isBusy() || rightFrontMotor.isBusy() || (System.currentTimeMillis() - startTime) < drivetime){
//            telemetry.addData("LF", leftFrontMotor.getCurrentPosition());
//            telemetry.update();

        //stoping the motors and resetting the encoders
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //sleep(250);
    }
//__________________________________________________________________________________________________
    public void DepotM () {

        //stopping the robot from trying to run the mission again
        boolean MissionComplete = true;
        if (tfod != null) {
            tfod.shutdown();
        }
        // Run boom motor up until it reaches upper limit
        while (l_limit.getState() == true) {
            boomMotor.setPower(-100);
        }
        boomMotor.setPower(0);    //turn off boom

        // bring boom back down to clear bolt
        boomMotor.setPower(0.2);
        sleep(900);
        boomMotor.setPower(0);
        //Unhook
        drivedeg(1450, 975, 0.25, 3000);
        // move towards depot
        drivedeg(3800, -3800, 0.25, 4000);
        // drop nacci
        servo1_pos = 0.2;
        servo1.setPosition(servo1_pos);
        sleep(1000);
        //backing away from element so we don't knock it out
        //  drivedeg(100, 100, 0.3, 1000);

        //Turn robot toward wall
        drivedeg(325, 350, 0.25, 2000);

        // reset servo position
        servo1_pos = 1;
        servo1.setPosition(servo1_pos);
        sleep(1000);
        //Turn towards wall
        drivedeg(200,-100,0.25,1000);

        //Backup to crater
        drivedeg(-4925, 6800, 0.3, 5000);

        // turn all motors off
        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
    }
//__________________________________________________________________________________________________
    public void DepotL () {

        //stopping the robot from trying to run the mission again
        boolean MissionComplete = true;
        if (tfod != null) {
            tfod.shutdown();
        }
        // Run boom motor up until it reaches upper limit
        while (l_limit.getState() == true) {
            boomMotor.setPower(-100);
        }
        boomMotor.setPower(0);    //turn off boom

        // bring boom back down to clear bolt
        boomMotor.setPower(0.2);
        sleep(900);
        boomMotor.setPower(0);

        //Unhook
        drivedeg(1070, 1070, 0.25, 2000);
        //drive too gold
        drivedeg(2000, -2000, 0.25, 3000);
        //drive too wall
        drivedeg(600,0,0.25,1000);
        //drive too depot
        drivedeg(1400,-1400,0.25,2500);
        // drop nacci
        servo1_pos = 0.2;
        servo1.setPosition(servo1_pos);
        sleep(1000);

        //turn towards crater
        drivedeg(400,100,0.25,1000);
        // backup out of depot
        drivedeg(-4400, 4400, 0.3, 5000);
        servo1_pos = 1;
        servo1.setPosition(servo1_pos);
        //sleep(1000);
        // turn all motors off
        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
    }
    //______________________________________________________________________________________________
    public void DepotR () {

        //stopping the robot from trying to run the mission again
        boolean MissionComplete = true;
        if (tfod != null) {
            tfod.shutdown();
        }
        // Run boom motor up until it reaches upper limit
        while (l_limit.getState() == true) {
            boomMotor.setPower(-100);
        }
        boomMotor.setPower(0);    //turn off boom

        // bring boom back down to clear bolt
        boomMotor.setPower(0.2);
        sleep(900);
        boomMotor.setPower(0);

        // Unhook
        drivedeg(1275, 925, 0.25, 2000);
        // turn towards gold
        drivedeg(700,0,0.25,1000);
        // drive towards gold
        drivedeg(1500,-1500,0.25,3000);
        // turn too depot
        drivedeg(0,-850,0.25,1000);
        // drive too depot
        drivedeg(1900,-1900,0.25,3000);
        // pivot back towards crater
        drivedeg(400,550,0.25,2000); // changed left from 700 to 600 to keep from hitting the wall

        // drop nacci
        servo1_pos = 0.2;
        servo1.setPosition(servo1_pos);
        sleep(1000);
        //turn away from wall
        drivedeg(20,-0,0.25,500);
        // backup to crater
        drivedeg(-4700, 4700, 0.3, 6000);

        servo1_pos = 1;
        servo1.setPosition(servo1_pos);
        sleep(1000);

        // turn all motors off
        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.40;
        tfodParameters.useObjectTracker = true;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    public void runOpMode() {
        // INITIALIZATION
        initVuforia();
        initTfod();
        // define the motor hardware map
        leftFrontMotor = hardwareMap.dcMotor.get("FL_drive");
        leftBackMotor = hardwareMap.dcMotor.get("RL_drive");
        rightFrontMotor = hardwareMap.dcMotor.get("FR_drive");
        rightBackMotor = hardwareMap.dcMotor.get("RR_drive");
        servo0 = hardwareMap.crservo.get("servo0");
        servo1 = hardwareMap.servo.get("servo1");
        boomMotor = hardwareMap.dcMotor.get("boomMotor");


        // define the sensor hardware

        // Lower Limit switch for the boom
        l_limit = hardwareMap.get(DigitalChannel.class, "l_limit");
        l_limit.setMode(DigitalChannel.Mode.INPUT);

        // Upper Limit Sensor for the Boom
        u_limit = hardwareMap.get(DigitalChannel.class, "u_limit");
        u_limit.setMode(DigitalChannel.Mode.INPUT);
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        boomMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        boomMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Status of the motor behavior
        boomMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);   // set this to brake or float

        // turn off all motor and reset servo positions to zero
        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
        boomMotor.setPower(0);
        servo0.setPower(0);

        // servo positions to 1 (upright)
        servo1_pos = 1;
        servo1.setPosition(servo1_pos);

        // output telemetry data to screen
        telemetry.addData("Mode", "waiting");
        telemetry.update();


        // AUTONOMOUS
        // transitioning to teleOp
        AutoTransitioner.transitionOnStop(this, "Fibby_TeleOp");
        // wait for the player to hit the start button
        //waitForStart();

        if (tfod != null) {
            tfod.activate();
        }

        //long startTime = System.currentTimeMillis(); //fetch starting time
        //while (false || (System.currentTimeMillis() - startTime) < 5000) {
        while (!isStarted()) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    if (updatedRecognitions.size() == 3) {
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
                        if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                            if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                telemetry.addData("Gold Mineral Position", "Left");
                                telemetry.update();
                                //DepotL();
                                goldPosition = 1;
                            } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                telemetry.addData("Gold Mineral Position", "Right");
                                telemetry.update();
                                //DepotR();
                                goldPosition = 3;
                            } else {
                                telemetry.addData("Gold Mineral Position", "Center");
                                telemetry.update();
                                //DepotM();
                                goldPosition = 2;
                            }
                        }
                    }
                }
            }
        }

        if (goldPosition == 3) DepotR();
        else if (goldPosition == 1) DepotL();
        else DepotM();

        //if (MissionComplete != true){

        //telemetry.addData("Gold Mineral Position", "Unknown");
        //telemetry.update();
        //DepotL();
        //}

        // turn all motors off
        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
    }
}