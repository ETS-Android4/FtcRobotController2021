package org.firstinspires.ftc.teamcode;
import android.graphics.Color;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Axis;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name="FibbyAuto22", group="Autonomous22")
public class FibbyAuto22 extends LinearOpMode {

    //light controls
    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;

    BNO055IMU imu;
    private DcMotor RF_drive = null;
    private DcMotor LF_drive = null;
    private DcMotor RB_drive = null;
    private DcMotor LB_drive = null;
    private DcMotor xAxis = null;
    private DcMotor yAxis = null;
    private DcMotor Intake = null;

    private DistanceSensor FrontDist;
    private DistanceSensor LeftDist;
    private DistanceSensor RightDist;
    private DistanceSensor BackDist;
    private DistanceSensor IntakeDist;

    private ElapsedTime runtime = new ElapsedTime();
    double timeleft;
    double IntakeTime;


    DigitalChannel xAxisStop;
    DigitalChannel yAxisStop;
    DigitalChannel yAxisTopStop;

    double LeftPower;
    double RightPower;
    double FrontPower;
    double BackPower;
    double xAxis_power = 0;
    double yAxis_power = 0;
    double Intake_power = 0;
    double heading = 0;
    double CorrectionFactor = 0.03;

    boolean Red_alliance = false;
    boolean FirstGyroRun = true;
    double rampmotorpower;
    boolean QuestionAnswered = false;
    boolean RunningToTheShop = false;
    boolean RunningToTheShopCam = false;
    boolean MadDuckPoints = false;
    boolean MadDuckPointsCam = false;
    boolean Warehouse = false;
    boolean Test = false;
    boolean RightTurn = false;
    boolean LeftTurn = false;
    boolean PANIC = false;
    float ElementCoordinates = 0;
    int ElementPosition = 0;
    int LastChance = 100;

    double dst_heading;
    public ArrayList allTrackables;
    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
           // "Ball",
            "Cube",
            "Duck",
           // "Marker"
    };

    private static final String VUFORIA_KEY =
            "Ae6MFyH/////AAABmf06rZVcN0VqtDgoLd1KtAcowILYnLgT+SXkGwocAFEpSEmZiGI51vbPAL/QfAgSIhIpPrAK3Fl+vReEgcd1kU5az1pYTI01VqIV1+3ELPbCEcnNMdw3Rs7L8tMMsyfY2nebMlSFfgn6rkfJnnoQEnr4gCGHB1K/7VVZsFg7AlG4SPJ1bORRlhkFf0xIP28Tvr80YnC06hnsL2AQgOJtGrPv7HUO04hWxe9jLMdwHhmnBu/FZfovI9A6EjrzB72joGNTzuLA5R2bBGKW6AsrkKcgw1y50EFOqanZ19gWBX7bc1OExeaNeNIMaGzbMV8jwVahVndqS4EiLc9FuudY21tw4b4jupvhSYUiSGBMtLmh\";\n";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    //-------------------------------------------------------------------------------------------------
    public void GyroDriveBase(boolean RampUp, int Course, double motorpower, boolean strafe, boolean right) {
        if (!Red_alliance)
            Course = Course * -1;
        if (FirstGyroRun) {
            FirstGyroRun = false;
            rampmotorpower = motorpower;
            //Set our starting motor powers before running ramp up
            if (rampmotorpower > 0 && RampUp)
                motorpower = 0.1;
            else if (rampmotorpower < 0 && RampUp)
                motorpower = -0.1;
        }
        checkOrientation();
        //Our gyro is upside down apparently, let's get it on it's feet!
        heading = heading * -1;
        //How far off course are we?? Store that in dst_heading
        dst_heading = (Course - heading);
        //Ramp up for motors
        if (motorpower < rampmotorpower && RampUp && rampmotorpower > 0)
            motorpower = motorpower + 0.03;
        else if (motorpower > rampmotorpower && RampUp && rampmotorpower < 0)
            motorpower = motorpower - 0.03;
        //Drive straight section
        if (!strafe) {
            //Set motor powers based on the direction we're moving and our heading
            //First half of if statement is for forward steering left, OR the last half is for reverse steering right
            if ((dst_heading < 0 && rampmotorpower > 0) || (dst_heading > 0 && rampmotorpower < 0)) {
                LeftPower = (motorpower - (dst_heading * CorrectionFactor * -1));
                RightPower = motorpower;
                //First half of if statement is for forward steering right , OR the last half is for reverse steering left
            } else if ((dst_heading > 0 && rampmotorpower > 0) || (dst_heading < 0 && rampmotorpower < 0)) {
                RightPower = (motorpower - (dst_heading * CorrectionFactor));
                LeftPower = motorpower;
            } else {
                LeftPower = motorpower;
                RightPower = motorpower;
            }
            RF_drive.setPower(-RightPower);
            LF_drive.setPower(LeftPower);
            RB_drive.setPower(-RightPower);
            LB_drive.setPower(LeftPower);
        } else if (strafe) {
            //Set motor powers based on the direction we're moving and our heading
            //First half of if statement is for strafing right and pivoting left, OR the last half is for reverse steering right
            if (dst_heading > 0 && !right) {
                FrontPower = (motorpower - (dst_heading * CorrectionFactor));
                BackPower = motorpower;
                //First half of if statement is for forward steering right , OR the last half is for reverse steering left
            } else if (dst_heading < 0 && !right) {
                BackPower = (motorpower - (dst_heading * CorrectionFactor * -1));
                FrontPower = motorpower;
            } else if (dst_heading < 0 && right) {
                FrontPower = (motorpower - (dst_heading * CorrectionFactor * -1));
                BackPower = motorpower;
                //First half of if statement is for forward steering right , OR the last half is for reverse steering left
            } else if (dst_heading > 0 && right) {
                BackPower = (motorpower - (dst_heading * CorrectionFactor));
                FrontPower = motorpower;
            } else {
                FrontPower = motorpower;
                BackPower = motorpower;
            }
            if (right) {
                RF_drive.setPower(FrontPower);
                LF_drive.setPower(FrontPower);
                RB_drive.setPower(-BackPower);
                LB_drive.setPower(-BackPower);
            } else {
                RF_drive.setPower(-FrontPower);
                LF_drive.setPower(-FrontPower);
                RB_drive.setPower(BackPower);
                LB_drive.setPower(BackPower);
            }
        }

    }

    //This function is used to consistently turn off powers to all of the motors and reset the encoders after every function.
    public void resetmotors() {
        RF_drive.setPower(0);
        LF_drive.setPower(0);
        RB_drive.setPower(0);
        LB_drive.setPower(0);

        RF_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        RF_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LF_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
    /*public void resetArm(){
        xAxis.setPower(0);
        yAxis.setPower(0);
        Intake.setPower(0);

        xAxis.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        yAxis.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        xAxis.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        yAxis.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }*/
    //-------------------------------------------------------------------------------------------------
    public void drivedist(int distance, double motorpower, int drivetime, boolean FrontSensor, int Course, boolean RampUp) {
        if (FrontSensor) {
            // Check front distance sensor to be sure we're not already too close
            if (FrontDist.getDistance(DistanceUnit.INCH) > distance) {
                // Leave motors powered until we close the distance to less than we passed in
                while ((FrontDist.getDistance(DistanceUnit.INCH) > distance) && (opModeIsActive())) {
                    GyroDriveBase(RampUp, Course, motorpower, false, false);
                }
            } else {



                // Leave motors powered until we close the distance to less than we passed in
                while ((FrontDist.getDistance(DistanceUnit.INCH) < distance) && (opModeIsActive())) {
                    GyroDriveBase(RampUp, Course, -motorpower, false, false);
                }
            }


        }
        //else is for using back sensor because the boolean frontsensor is false.
        else {
            // Check back distance sensor to be sure we're not already too close
            if (BackDist.getDistance(DistanceUnit.INCH) > distance) {
                // Leave motors powered until we close the distance to less than we passed in
                while ((BackDist.getDistance(DistanceUnit.INCH) > distance) && (opModeIsActive())) {
                    GyroDriveBase(RampUp, Course, -motorpower, false, false);
                }
            } else {
                // Leave motors powered until we expand the distance to more than we passed in
                while ((BackDist.getDistance(DistanceUnit.INCH) < distance) && (opModeIsActive())) {
                    GyroDriveBase(RampUp, Course, motorpower, false, false);
                }
            }
        }
        //turn off motors
        resetmotors();
        //pause to give motors a rest NO GRINDING GEARS
        FirstGyroRun = true;
       // sleep(drivetime);
    }

    //-------------------------------------------------------------------------------------------------
    public void SFdist(int dist, double motorpower, boolean distR, boolean StopMotors, boolean Right, int Course, boolean RampUp) {
        //If we want to use the LEFT distance sensor, then it will go through each of the following while loops to determine which direction to move.
        if (distR == false) {
            //This will strafe left using the left distance sensor moving TOWARD the object on the left
            if (!Right)
                while ((LeftDist.getDistance(DistanceUnit.INCH) >= dist) && (opModeIsActive())) {
                    GyroDriveBase(RampUp, Course, motorpower, true, Right);
                }
                //This will strafe right using the left distance sensor moving AWAY from the object on the left
            else while ((LeftDist.getDistance(DistanceUnit.INCH) <= dist) && (opModeIsActive())) {
                GyroDriveBase(RampUp, Course, motorpower, true, Right);
            }

        }
//Use the right Distance Sensor
        if (distR == true) {
            if (Right) {
                while ((RightDist.getDistance(DistanceUnit.INCH) >= dist) && (opModeIsActive())) {
                    // Strafe Right to close the gap
                    GyroDriveBase(RampUp, Course, motorpower, true, Right);
                }}
            else {
                telemetry.addLine("2nd Else in right distance sensor strafe");
                telemetry.addData("Right (IN)", String.format("%.01f in", RightDist.getDistance(DistanceUnit.INCH)));
                telemetry.update();
                //sleep(3000);
                while ((RightDist.getDistance(DistanceUnit.INCH) <= dist) && (opModeIsActive())) {
                    //Strafe Left to increase the gap
                    telemetry.addLine("SUCCESS, Go robot go!");
                    telemetry.update();
                   // sleep(2000);
                    GyroDriveBase(RampUp, Course, motorpower, true, Right);
                }
            }
        }

//turn off motors
        if (StopMotors)
            resetmotors();
        FirstGyroRun = true;
    }

    //-------------------------------------------------------------------------------------------------
    public void drivedeg(int drivedegrees, double motorpower, int drivetime, boolean StopMotors, int Course, boolean RampUp) {
        RF_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        RF_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LF_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FirstGyroRun = true;

        //The negative for the right motor to move in the correct direction

        //If we're drive backwards, we're looking at negative degrees and need to switch the operator in the while loop
        if (drivedegrees < 0) {
            while ((LF_drive.getCurrentPosition() >=  drivedegrees) && (LB_drive.getCurrentPosition() >=  drivedegrees)&& (RB_drive.getCurrentPosition() <=  -drivedegrees) && (RF_drive.getCurrentPosition() <=  -drivedegrees) && (opModeIsActive())) {
                GyroDriveBase(RampUp,Course,motorpower, false, false);
            }}
        else {
            while (((LF_drive.getCurrentPosition() <=  drivedegrees) && (LB_drive.getCurrentPosition() <=  drivedegrees)&& (RB_drive.getCurrentPosition() >=  -drivedegrees) && (RF_drive.getCurrentPosition() >=  -drivedegrees)) && (opModeIsActive())) {
                GyroDriveBase(RampUp,Course,motorpower, false, false);

            }}

        telemetry.addData("Encoders RF RB LF LB", "%7d :%7d :%7d :%7d",
                RF_drive.getCurrentPosition(),
                RB_drive.getCurrentPosition(),
                LF_drive.getCurrentPosition(),
                LB_drive.getCurrentPosition());
        telemetry.update();


//turn off motors
        if (StopMotors)
            resetmotors();
        //pause to give motors a rest NO GRINDING GEARS
        sleep(drivetime);
        FirstGyroRun=true;
    }
    //-------------------------------------------------------------------------------------------------
//Strafe using the motor encoders
    public void SF(int deg, double motorpower, int drivetime, boolean Right, boolean StopMotors, int Course, boolean RampUp) {
        RF_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        RF_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LF_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//Check to see if we're strafing right or left then run until encoder values are met then exit.
        if (Right)
            while (((LF_drive.getCurrentPosition() <=  deg) && (RF_drive.getCurrentPosition() <=  deg)&& (LB_drive.getCurrentPosition() >=  -deg) && (RB_drive.getCurrentPosition() >=  -deg)) && (opModeIsActive())) {
                GyroDriveBase(RampUp,Course,motorpower,true,Right);
            }
        else
            while (((LB_drive.getCurrentPosition() <=  deg) && (RB_drive.getCurrentPosition() <=  deg)&& (RF_drive.getCurrentPosition() >=  -deg) && (LF_drive.getCurrentPosition() >=  -deg)) && (opModeIsActive())) {
                GyroDriveBase(RampUp,Course,motorpower,true,Right);
            }

//turn off motors
        if(StopMotors)
            resetmotors();
       // sleep(drivetime);
        FirstGyroRun = true;
    }

    //-------------------------------------------------------------------------------------------------
    public void SpinGyro (double dest_heading, double motorpower, long drivetime, boolean Right, boolean ResetMotors) {
        if (Red_alliance) dest_heading = dest_heading * -1;
        long starttime = System.currentTimeMillis();
        checkOrientation();
        //If Right boolean is true it will spin right
        if (Right) {
            // Turn on the motors
            LF_drive.setPower(motorpower);
            RF_drive.setPower(motorpower);
            LB_drive.setPower(motorpower);
            RB_drive.setPower(motorpower);
            //Wait for heading to be achieved - if heading can't be achieved, timeout and exit the loop
            while ((heading >= dest_heading) && (System.currentTimeMillis() <= starttime+drivetime) && opModeIsActive()) {
                checkOrientation();
            }
        }
        //If false it will spin left
        else {
            // Turn on the motors
            LF_drive.setPower(-motorpower);
            RF_drive.setPower(-motorpower);
            LB_drive.setPower(-motorpower);
            RB_drive.setPower(-motorpower);
            while ((heading <= dest_heading) &&  (System.currentTimeMillis() <= starttime+drivetime) && (opModeIsActive())) {
                checkOrientation();
            }
        }

        //turn off motors if the boolean ResetMotors is true.
        if (ResetMotors)
            resetmotors();
        FirstGyroRun = true;
    }
    //------------------------------------------------------------------------------------------------------------------------------
    public void SFtime(int drivetime, double motorpower, boolean Right, boolean StopMotors, int Course, boolean RampUp){
        long starttime = System.currentTimeMillis();
        //This will strafe left
        while ((System.currentTimeMillis() <= starttime+drivetime) && (opModeIsActive())) {
            GyroDriveBase(RampUp,Course,motorpower,true, Right);
        }
        //turn off motors
        if (StopMotors)
            resetmotors();
        FirstGyroRun = true;
    }
    //------------------------------------------------------------------------------------------------------------------------------
    public void Drivetime(double motorpower, int drivetime, boolean StopMotors, int Course, boolean RampUp){
        long starttime = System.currentTimeMillis();
        //Run Forwards
        while ((motorpower > 0) && (System.currentTimeMillis() <= starttime+drivetime) && (opModeIsActive())) {
            GyroDriveBase(RampUp, Course, motorpower, false, false);
        }
        //Run backwards
        while ((motorpower<0) && (System.currentTimeMillis() <= starttime+drivetime) && (opModeIsActive())) {
            GyroDriveBase(RampUp, Course, motorpower, false, false);
        }

//turn off motors if the boolean StopOnLine is true
        if (StopMotors)
            resetmotors();
        FirstGyroRun = true;
    }
    //------------------------------------------------------------------------------------------------------------------------------
private void MoveXAxisDeg (int deg, double motorPower,int time) {
      xAxis.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        xAxis.setTargetPosition(deg);

        xAxis.setPower(motorPower);

        xAxis.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    if ((xAxis.getCurrentPosition() >= deg-5) && (xAxis.getCurrentPosition() <= deg+5))//Are we there yet??
    {
        xAxis.setPower(0);
    }

        //sleep(time);

    //set motor to 0 after in run
       // xAxis.setPower(0);

        //xAxis.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
}
    //--------------------------------------------------------------------------------------------------------------------------

    private void MoveYAxisDeg (int deg,double motorPower,int time) {
        yAxis.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        yAxis.setTargetPosition(deg);
        yAxis.setPower(motorPower);
        yAxis.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if ((yAxis.getCurrentPosition() >= deg-5) && (yAxis.getCurrentPosition() <= deg+5))//Are we there yet??
        {
            yAxis.setPower(0);
        }
        //sleep(time);

        //yAxis.setPower(0);

        //yAxis.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    //-------------------------------------------------------------------------------------------------------------------------
    //(ArrayList<VuforiaTrackable> allTrackables, TOOK THIS OUT WILL WANT LATER
    //start of our runs!
    public void RunningToTheShop (boolean Red_Alliance) {

        boolean RightTurn = true;
        boolean LeftTurn = false;
        if (!Red_alliance) {
            RightTurn = false;
            LeftTurn = true;
        }

        // lift arm up y axis
        MoveYAxisDeg(2100,0.4,1000);
        // strafe off wall 15 inches
        SFdist(15, 0.5, RightTurn, true, LeftTurn, 0, true);
        // drive backwards
        if(Red_Alliance)
        {
            drivedeg(-500, -0.4 , 1000, true, 0, true);
        }
        else if (!Red_Alliance)
        {
            drivedeg(-700, -0.4 , 1000, true, 0, true);
        }
        // strafe farther from wall 25 inches
        // turn arm towards the hub
        if(Red_Alliance)
        {
            SFdist(24, 0.5, RightTurn, true, LeftTurn, 0, true);
            MoveXAxisDeg(-1630,0.4,1000);
        }
        else if (!Red_Alliance)
        {
            SFdist(26, 0.5, RightTurn, true, LeftTurn, 0, true);
            MoveXAxisDeg(1630,0.4,1000);
        }
        sleep(3000);
        // drop waffle
        Intake.setPower(-.7);
        sleep(500);
        Intake.setPower(0);
        // set arm x axis to 0
        MoveXAxisDeg(-2,0.4,1000);
        // strafe to wall 15 inches
        SFdist(18, 0.5, RightTurn, true, RightTurn, 0, true);
        // drive forward
        drivedeg(450, 0.4 , 1000, true, 0, true);
        // strafe to wall 1 inches
        SFdist(1, 0.5, RightTurn, true, RightTurn, 0, true);
        // drive into the warehouse
        drivedeg(2000, .4, 1000, true, 0, true);
        // strafe to the left
        if(Red_Alliance)
        {
            SFdist(26, .4, RightTurn, true, LeftTurn, 0, true);
        }
        else if(!Red_Alliance)
        {
            SFdist(28, .4, RightTurn, true, LeftTurn, 0, true);
        }
        // Im in spain without the a
        if(Red_Alliance)
        {
            SpinGyro(-90, .4, 10000, true, false);
            SFdist(3, 0.4, false, true, false, 90, true);
        }
        else if(!Red_Alliance)
        {
            SpinGyro(-90, .4, 10000, false, false);
            SFdist(3, 0.4, true, true, true, 90, true);
        }
        // set arm y axis down
        MoveYAxisDeg(1100,0.4,1000);
        sleep(1000);

    }
//------------------------------------------------------------------------------------------------------------------------------
    public void test (boolean Red_Alliance) {
        //if red
        boolean RightTurn = true;
        boolean LeftTurn = false;
        //if blue
        if (!Red_alliance) {
            RightTurn = false;
            LeftTurn = true;
        }
       // drivedist(5,0.5,1000,false,0,true);

        SFdist(30,0.5,LeftTurn,true,RightTurn,-90,true);


    }

    public void MadDuckPointsCam (boolean Red_Alliance) {
        //if red
        boolean RightTurn = true;
        boolean LeftTurn = false;
        //if blue
        if (!Red_alliance) {
            RightTurn = false;
            LeftTurn = true;
        }
if (!Red_Alliance && ElementPosition == 1)
{ElementPosition = 3;}
else if (!Red_Alliance && ElementPosition == 2)
{ElementPosition = 1;}
else if (!Red_Alliance && ElementPosition == 3)
{ElementPosition = 2;}
else
{}
    if (ElementPosition == 1)
    {
        if (Red_Alliance)
        drivedist(18, 0.7, 10000, false, 0, true);
        // move forward
        //if (!Red_Alliance)
        //drivedist(30, 0.6, 10000, false, 0, true);
        // lift arm to bottom hub
        MoveYAxisDeg(800, 0.7, 1000);
        if (!Red_Alliance)
        drivedist(28, 0.7, 10000, false, 0, true);
        // turn towards hub 45 degrees
        SpinGyro(62, 0.4, 10000, RightTurn, false);
         drivedeg(100, 0.3, 1000, true, 65, true);
        // shoots block into hub
        Intake.setPower(-1);
        sleep(1000);
        Intake.setPower(0);
        drivedeg(-100,-0.7, 1000, true, 55, true);
        SpinGyro(0, 0.4, 10000, LeftTurn, false);
        drivedist(14, 0.7, 10000, false, 0, true);
        MoveYAxisDeg(1400, 0.7, 1000);
        sleep(1100);
    }
    else if (ElementPosition == 2)
    {
        // move forward
        drivedist(30, 0.7, 10000, false, 0, true);
        // lift arm to middle hub
        MoveYAxisDeg(1200, 0.7, 1000);
        // sleep
        sleep(1500);
        // turn arm 45 degrees
        if (Red_Alliance) {
            MoveXAxisDeg(700, .7, 1000);//1630 0.5
        } else if (!Red_Alliance) {
            MoveXAxisDeg(-700, 0.7, 1000);//-1630 0.5
        }

        // sleep
        sleep(1500);
        // shoots block into hub
        Intake.setPower(-1);
        sleep(1500);
        Intake.setPower(0);
        MoveXAxisDeg(0, 0.7, 1000);// 0.5
        sleep(1100);
        MoveYAxisDeg(1400, 0.7, 1000);
        sleep(1100);
        drivedist(14, 0.5, 10000, false, 0, true);
    }
    else if (ElementPosition == 3)
    {
        // lift arm to top hub
        MoveYAxisDeg(2100, 0.8, 1000);
        // move forward was 30
        drivedist(28, 0.7, 10000, false, 0, true);
        // sleep
        //sleep(1500);
        // turn arm 45 degrees
        if (Red_Alliance) {
            MoveXAxisDeg(650, .7, 1000);//1630 0.5
        } else if (!Red_Alliance) {
            MoveXAxisDeg(-650, 0.7, 1000);//-1630 0.5
        }
        // sleep
        sleep(1500);
        // shoots block into hub
        Intake.setPower(-1);
        sleep(1500);
        Intake.setPower(0);
        MoveXAxisDeg(0, 0.7, 1000);//0.5
        sleep(1300);
        MoveYAxisDeg(1400, 0.7, 1000);
        sleep(1100);
        drivedist(14, 0.5, 10000, false, 0, true);
    }

    //SpinGyro(90, 0.4, 1000, RightTurn, true);
    Drivetime(-0.6,1000,true,90,true);
    if (Red_Alliance){}
    else
        MoveXAxisDeg(-240,0.5,10000);
    drivedist(3, 0.25, 10000, false, 90, true);
    // back up more
    Drivetime(-0.25,400,true,90,true);
    if (Red_Alliance)
    SFdist(7, 0.4, RightTurn, true, RightTurn, 90, true);
    else
        SFdist(12, 0.4, RightTurn, true, RightTurn, 90, true);
    Intake.setPower(0.45);
    sleep(3500);
    Intake.setPower(0);
        MoveXAxisDeg(0, 0.7, 1000);
        MoveYAxisDeg(5, 0.3, 1000);


    if (Warehouse){

        SFdist(50, 0.7, RightTurn, false, LeftTurn, 90, true);
        SF(400, 0.7, 1000, LeftTurn, true, 90, true);
        drivedist(50, 0.7, 1000, false, 90, true);
        drivedeg(450, 0.3, 1000, false, 90, false);
        SF(1500, 0.4, 1000, RightTurn, true, 90, true);
        drivedist(15, 1, 2000, true, 90, false);
    }
    else {
        SFdist(28, 0.4, RightTurn, true, LeftTurn, 90, true);
        Drivetime(-0.25,400,true,90,true);
    }
}

    public void RunningToTheShopCam (boolean Red_Alliance) {

        boolean RightTurn = true;
        boolean LeftTurn = false;
        if (!Red_alliance) {
            RightTurn = false;
            LeftTurn = true;
        }
        if (Red_Alliance && ElementPosition == 1) {
            ElementPosition = 3;
        } else if (Red_Alliance && ElementPosition == 2) {
            ElementPosition = 1;
        } else if (Red_Alliance && ElementPosition == 3) {
            ElementPosition = 2;
        } else {
        }
        //while ((!PANIC) || (!opModeIsActive())) {
        // FIX AS SOON AS POSIBLE
            if (ElementPosition == 1) {
                // lift arm to bottom hub
                MoveYAxisDeg(820, 0.7, 1000);
                drivedist(4, 0.6, 1000,false, 0, true);
               // if (Red_Alliance)
                SpinGyro(-90,0.5,1000,LeftTurn,true);
                SF(700, 0.5, 1000, RightTurn, true, -90, true);
                //SFdist(30,0.5,LeftTurn,true,RightTurn,-90,true);
                // turn towards hub 45 degrees
                SpinGyro(-74, 0.5, 10000, RightTurn, false);
                drivedeg(115, 0.25, 1000, true, -72, true);
                // shoots block into hub
                Intake.setPower(-0.8);
                sleep(300);
                while ((IntakeDist.getDistance(DistanceUnit.INCH) <= 2.5) && (opModeIsActive())) {
                    Intake.setPower(-0.8);
                }
                //sleep(1100);
                Intake.setPower(0);
                drivedeg(-100, -0.7, 1000, true, -45, true);
                SpinGyro(0, 0.6, 10000, RightTurn, false);
                //sleep(1100);
                if (Red_Alliance)
                    drivedist(23, 0.8, 10000, false, 0, true);
                if (!Red_Alliance)
                    drivedist(23, 0.8, 10000, false, 0, true);
            } else if (ElementPosition == 2) {
                // lift arm to middle hub
                MoveYAxisDeg(1350, 0.7, 1000);
                // move forward
                drivedist(27, 0.5, 10000, false, 0, true);
                // sleep
                //  sleep(100);
                // turn arm 45 degrees
                if (!Red_Alliance) {
                    MoveXAxisDeg(470, .6, 1000);//1630 0.7
                } else if (Red_Alliance) {
                    MoveXAxisDeg(-500, .6, 1000);//-1630 0.7
                }

                // sleep
                sleep(1500);
                // shoots block into hub
                Intake.setPower(-0.8);
                sleep(300);
                while ((IntakeDist.getDistance(DistanceUnit.INCH) <= 2.5) && (opModeIsActive())) {
                    Intake.setPower(-0.8);
                }
                //sleep(1100);
                Intake.setPower(0);
                MoveXAxisDeg(0, 0.7, 1000);//0.6
                drivedist(30, 0.5, 10000, false, 0, true);
                //sleep(500);
                MoveYAxisDeg(1400, 0.5, 1000);
                //sleep(1100);
            } else if (ElementPosition == 3) {
                // lift arm to top hub
                MoveYAxisDeg(2100, 0.6, 1000);
                // move forward was 27
                if (Red_Alliance)
                    drivedist(27, 0.6, 10000, false, 0, true);
                else
                    drivedist(32, 0.6, 10000, false, 0, true);
                // sleep
                //sleep(1500);
                // turn arm 45 degrees
                if (!Red_Alliance) {
                    MoveXAxisDeg(600, .7, 1000);//1630 0.6
                } else if (Red_Alliance) {
                    MoveXAxisDeg(-600, .7, 1000);//-1630 .6
                }
                // sleep
                sleep(1500);
                // shoots block into hub
                Intake.setPower(-0.8);
                sleep(300);
                while ((IntakeDist.getDistance(DistanceUnit.INCH) <= 2.5) && (opModeIsActive())) {
                    Intake.setPower(-0.8);
                }
                //sleep(1100);
                Intake.setPower(0);
                MoveXAxisDeg(0, 0.7, 1000);//0.6
                drivedist(30, 0.5, 10000, false, 0, true);
                //sleep(500);
                MoveYAxisDeg(1400, 0.5, 1000);
                //sleep(1100);
            }
            //Strafe to the wall
            SFdist(3, 0.6, RightTurn, true, RightTurn, 90, true);
            //Move arm down
            MoveYAxisDeg(1, 0.5, 5000);
            // drive into warehouse and pick up object //was 87
            //block 2
            IntakeTime = 5;
            drivedeg(300, 0.4, 700, false, 93, true);
            while ((IntakeDist.getDistance(DistanceUnit.INCH) >= 2.5) && (opModeIsActive())) {
                //IntakeTime = 25;
                Intake.setPower(1);
                IntakeTime = IntakeTime - 0.1;
                if (IntakeTime > 0)
                    Drivetime(0.30, 10, false, 90, false);
                else if ((IntakeTime <= 0 )&&( IntakeTime > -5)) {
                    Drivetime(-0.20, 250, true, 90, false);
                      Drivetime(0.20, 300, true, 80, false);
                    //   Drivetime(-0.20, 40, true, -30, false);
                }
               // else if (IntakeTime <= -3){
                 //   PANIC = true;
               // }
            }
            Intake.setPower(0);
            // move arm up
            MoveYAxisDeg(2100, 0.5, 2000);
            //
            drivedist(40, 0.8, 100000, true, 83, true);
            //
            if (!Red_Alliance)
                drivedeg(-350, -0.9, 0, true, 86, true);
            else
                drivedeg(-350, -0.9, 0, true, 86, true);
            // move arm towards hub
            if (!Red_Alliance) {
                MoveXAxisDeg(750, .7, 10000);//1630 0.6
            } else if (Red_Alliance) {
                MoveXAxisDeg(-750, .7, 10000);//1630 0.6
            }
            //sleep(1000);
            // strafe towards hub
        if (!Red_Alliance)
            SFdist(26, 0.6, RightTurn, true, LeftTurn, 90, true);
        if (Red_Alliance)
            SFdist(20, 0.6, RightTurn, true, LeftTurn, 90, true);
            // shoot object out
        Intake.setPower(-0.8);
        sleep(300);
            while ((IntakeDist.getDistance(DistanceUnit.INCH) <= 2.5) && (opModeIsActive())) {
                Intake.setPower(-0.8);
                //sleep(100);
            }
            //sleep(1100);
            Intake.setPower(0);
            // move arm to middle
            MoveXAxisDeg(0, 0.5, 2000);//0.7
            // strafe to wall
            SFdist(3, .6, RightTurn, true, RightTurn, 90, true);
            // move arm down
            MoveYAxisDeg(2, 0.6, 1000);
            // drive into the warehouse the second time block 3
            drivedeg(900, 0.4, 1000, false, 92, true);
            IntakeTime = 2;
            while ((IntakeDist.getDistance(DistanceUnit.INCH) >= 2.5) && (opModeIsActive())) {
                //IntakeTime = 25;
                Intake.setPower(1);
                IntakeTime = IntakeTime - 0.1;
                if (IntakeTime > 0)
                    Drivetime(0.30, 10, false, 90, false);
                else if (IntakeTime <= 0) {
                    Drivetime(-0.20, 250, true, 90, true);
                    Drivetime(0.20, 300, true, 80, true);
                    //   Drivetime(-0.20, 40, true, -30, false);
                }
                //else if (IntakeTime <= -2){
                //    PANIC = true;
               // }
            }
            Intake.setPower(0);
            // move arm up
            MoveYAxisDeg(2100, 0.5, 2000);
            //
            drivedist(40, 0.8, 100000, true, 80, true);
            //
            if (!Red_Alliance)
                drivedeg(-350, -0.9, 0, true, 86, true);
            else
                drivedeg(-350, -0.9, 0, true, 86, true);
            // move arm towards hub
            if (!Red_Alliance) {
                MoveXAxisDeg(750, .7, 10000);//1630 0.6
            } else if (Red_Alliance) {
                MoveXAxisDeg(-750, .7, 10000);//1630 0.6
            }
            //sleep(1000);
            // strafe towards hub
        if (!Red_Alliance)
            SFdist(26, 0.6, RightTurn, true, LeftTurn, 90, true);
        //if (Red_Alliance)
           // SFdist(20, 0.6, RightTurn, true, LeftTurn, 90, true);
            // shoot object out
        Intake.setPower(-0.8);
        sleep(300);
            while ((IntakeDist.getDistance(DistanceUnit.INCH) <= 2.5) && (opModeIsActive())) {
                Intake.setPower(-0.8);
                sleep(100);
            }
            //sleep(1100);
            Intake.setPower(0);
            // move arm to middle
            MoveXAxisDeg(0, 0.6, 2000);//0.7
            // strafe to wall
            SFdist(3, .6, RightTurn, true, RightTurn, 90, true);
            // move arm down
            MoveYAxisDeg(1, 0.6, 1000);
            // block 4
            drivedeg(900, 0.4, 1000, false, 92, true);
            IntakeTime = 5;
            while ((IntakeDist.getDistance(DistanceUnit.INCH) >= 2.5) && (opModeIsActive())){
                //IntakeTime = 25;
                Intake.setPower(1);
                IntakeTime = IntakeTime - 0.1;
                if (IntakeTime > 0)
                    Drivetime(0.30, 10, false, 90, false);
                else if ((IntakeTime <= 0 )&&( IntakeTime > -5)) {
                    Drivetime(-0.20, 250, true, 90, false);
                    Drivetime(0.20, 300, true, 80, false);
                 //   Drivetime(-0.20, 40, true, -30, false);
                }
               // else if (IntakeTime <= -2) {
                 //   PANIC = true;
              //  }
            }
            Intake.setPower(0);
            // move arm up
            MoveYAxisDeg(2100, 0.5, 2000);
            //
            drivedist(40, 0.8, 100000, true, 80, true);
            //
            if (!Red_Alliance)
                drivedeg(-350, -0.9, 0, true, 86, true);
            else// if red alliance
                drivedeg(-350, -0.9, 0, true, 86, true);
            // move arm towards hub
            if (!Red_Alliance) {
                MoveXAxisDeg(750, .7, 10000);//1630 0.6
            } else if (Red_Alliance) {
                MoveXAxisDeg(-750, .7, 10000);//1630 0.6
            }
            //sleep(1000);
            // strafe towards hub
        if (!Red_Alliance)
            SFdist(26, 0.6, RightTurn, true, LeftTurn, 90, true);
        if (Red_Alliance)
            SFdist(20, 0.6, RightTurn, true, LeftTurn, 90, true);
            // shoot object out
        Intake.setPower(-0.8);
        sleep(300);
            while ((IntakeDist.getDistance(DistanceUnit.INCH) <= 2.5) && (opModeIsActive())) {
                Intake.setPower(-0.8);
                sleep(100);
            }
            //sleep(1100);
            Intake.setPower(0);
            // move arm to middle
            MoveXAxisDeg(0, 0.6, 2000);//0.7
            // strafe to wall
            SFdist(3, .6, RightTurn, true, RightTurn, 90, true);
            // move arm down
            MoveYAxisDeg(1, 0.6, 1000);
            // drive into the warehouse
            if (!Red_Alliance)
                drivedist(20, 1, 1000, true, 92, true);
            else
                drivedist(20, 1, 1000, true, 92, true);
            //PANIC = true;
        }


//--------------------------------------------------------------------------------------------------------------------------------
    private void checkOrientation() {
// read the orientation of the robot
        angles = this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        this.imu.getPosition();
// and save the heading
        heading = angles.firstAngle;
    }
    //------------------------------------------------------------------------------------------------------------------------------
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }
    //------------------------------------------------------------------------------------------------------------------------------
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.7f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
    //------------------------------------------------------------------------------------------------------------------------------
    public void runOpMode() {
        //Prepare IMU to read gyro
        BNO055IMU.Parameters gyro_parameters = new BNO055IMU.Parameters();
        gyro_parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        gyro_parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        gyro_parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        gyro_parameters.loggingEnabled      = true;
        gyro_parameters.loggingTag          = "IMU";
        gyro_parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(gyro_parameters);

        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1, 16.0/9.0);
            tfod.setClippingMargins(0,100,0,0);
        }
        timeleft = 30 - runtime.seconds();
       // timeleft = 30;
        //IntakeTime = 30 - runtime.seconds();
       // IntakeTime = 30;

       /* if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());

                    i++;
                }
                telemetry.update();
            }
        }
*/
        //Mapping motor to variable names
        RF_drive = hardwareMap.dcMotor.get("RF_drive");
        LF_drive = hardwareMap.dcMotor.get("LF_drive");
        RB_drive = hardwareMap.dcMotor.get("RB_drive");
        LB_drive = hardwareMap.dcMotor.get("LB_drive");
        xAxis = hardwareMap.dcMotor.get("xAxis");
        yAxis = hardwareMap.dcMotor.get("yAxis");
        Intake = hardwareMap.dcMotor.get("Intake");

        // LED LIGHTS
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

        LeftDist = hardwareMap.get(DistanceSensor.class, "LeftDist");

        RightDist = hardwareMap.get(DistanceSensor.class, "RightDist");

        FrontDist = hardwareMap.get(DistanceSensor.class, "FrontDist");

        BackDist = hardwareMap.get(DistanceSensor.class, "BackDist");

        IntakeDist = hardwareMap.get(DistanceSensor.class, "IntakeDist");

        xAxisStop = hardwareMap.get(DigitalChannel.class, "xAxisStop");
        xAxisStop.setMode(DigitalChannel.Mode.INPUT);
        yAxisStop = hardwareMap.get(DigitalChannel.class, "yAxisStop");
        yAxisStop.setMode(DigitalChannel.Mode.INPUT);
        yAxisTopStop = hardwareMap.get(DigitalChannel.class, "yAxisTopStop");
        yAxisStop.setMode(DigitalChannel.Mode.INPUT);
        //DuckSpinner = hardwareMap.dcMotor.get("DuckSpinner");

        RF_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LF_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        xAxis.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        yAxis.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       // DuckSpinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        AutoTransitioner.transitionOnStop(this, "FibbyTeleOp22");
        if (yAxisStop.getState()==true) //Is the arm all the way down on the limit switch?
            while (xAxisStop.getState()== false)  xAxis.setPower(0.2);//Move the arm right until the limit switch is triggered
        xAxis.setPower(0);
        xAxis.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        xAxis.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //

        xAxis.setTargetPosition(-25);
        xAxis.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        xAxis.setPower(0.2);
        while(xAxis.getCurrentPosition() >=-15)
        {}
        xAxis.setPower(0);
        xAxis.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        yAxis.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        xAxis.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        yAxis.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("ALLIANCE: Red or Blue?");
        telemetry.update();
        while (QuestionAnswered == false) {
            if (gamepad1.b) {
                Red_alliance = true;
                QuestionAnswered = true;
                pattern = RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_RED;
                blinkinLedDriver.setPattern(pattern);
            }
            else if (gamepad1.x){
                Red_alliance = false;
                QuestionAnswered = true;
                pattern = RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_BLUE;
                blinkinLedDriver.setPattern(pattern);
            }
        }

        QuestionAnswered = false;
        telemetry.addLine(" | Fibonacci Autonomous Run | ");
        telemetry.addLine("Y:                            ");
        telemetry.addLine("A: Test                       ");
        telemetry.addLine("X: MadDuckPointsCam           ");
        telemetry.addLine("B: RunningToTheShopCam        ");
        telemetry.update();
        sleep (1000);
        while (QuestionAnswered == false) {
            if (gamepad1.y) {
                //
                telemetry.addLine("wrong bad");
                RunningToTheShop = true;
                QuestionAnswered = true;
            }
            if (gamepad1.a){
                //
                telemetry.addLine("Test");
                Test = true;
                QuestionAnswered = true;
            }
            if (gamepad1.x) {
                //MadDuckPointsCam
                telemetry.addLine("MadDuckPointsCam");
                while (QuestionAnswered == false){
                    MadDuckPointsCam = true;
                    telemetry.addLine("| Warehouse |");
                    telemetry.addLine("    A: Yes   ");
                    telemetry.addLine("    B: No    ");
                    telemetry.update();
                    if (gamepad1.a){
                        Warehouse = true;
                        QuestionAnswered = true;
                    }
                    else if (gamepad1.b){
                        Warehouse = false;
                        QuestionAnswered = true;
                    }
                }
                QuestionAnswered = true;
            }
            if (gamepad1.b) {
                //RunningToTheShopCam
                telemetry.addLine("RunningToTheShopCam");
                RunningToTheShopCam = true;
                QuestionAnswered = true;
            }
            telemetry.update();
        }

        while (!isStarted()) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());

                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        ElementCoordinates = recognition.getLeft();
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());

                        i++;
                    }
                    telemetry.update();
                    if (i==0)
                        ElementCoordinates=0;
                }
            }

        }
        tfod.deactivate();//turn off camera
        if (ElementCoordinates > 275)
            ElementPosition = 3;
        else if (ElementCoordinates <=275 && ElementCoordinates > 0)
            ElementPosition = 2;
        else
            ElementPosition =1;

       switch (ElementPosition){
           case 1:{pattern = RevBlinkinLedDriver.BlinkinPattern.AQUA;
               blinkinLedDriver.setPattern(pattern); }
               break;
           case 2:{pattern = RevBlinkinLedDriver.BlinkinPattern.HOT_PINK;
               blinkinLedDriver.setPattern(pattern);}
               break;
           case 3:{pattern = RevBlinkinLedDriver.BlinkinPattern.VIOLET;
               blinkinLedDriver.setPattern(pattern);}
               break;
       }

        if (RunningToTheShop)
        {// took out old auto becalse we have changed our motors and this would break our robot to run so i am not going to give the option
         //   RunningToTheShop(Red_alliance);
        }
        else if (Test){
         test(Red_alliance);
        }
        else if (MadDuckPointsCam){
            MadDuckPointsCam(Red_alliance);
        }
        else if (RunningToTheShopCam){
            RunningToTheShopCam(Red_alliance);
        }

        //turn off motors
        resetmotors();
    }
}