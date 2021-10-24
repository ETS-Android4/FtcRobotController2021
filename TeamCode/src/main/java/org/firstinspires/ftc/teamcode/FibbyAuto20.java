
package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.security.KeyStore;
import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import org.firstinspires.ftc.robotcontroller.external.samples.SampleRevBlinkinLedDriver;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;


import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.vuforia.CameraDevice;
import com.vuforia.Vuforia;

import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
@Disabled
@Autonomous(name="FibbyAuto20", group="Autonomous1")

public class FibbyAuto20 extends LinearOpMode {
    ColorSensor colorSensor;
    Telemetry.Item patternName;
    Telemetry.Item display;
//    SampleRevBlinkinLedDriver.DisplayKind displayKind;
    Deadline ledCycleDeadline;
    Deadline gamepadRateLimit;

    protected enum DisplayKind {
        MANUAL,
        AUTO
    }
    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;
    BNO055IMU imu;
    private DcMotor FL_drive = null;
    private DcMotor FR_drive = null;
    private DcMotor BL_drive = null;
    private DcMotor BR_drive = null;
    private DcMotor The_Boom = null;
    private DcMotor The_Boom2 = null;
    private DcMotor Reach = null;
    private DcMotor Boom_Arm = null;
    private Servo servo = null;
    private Servo left_chop = null;
    private Servo right_chop = null;
    //private Servo blinkin = null;
    private DistanceSensor distleft;
    private DistanceSensor distright;
    private DistanceSensor distfront;
    private DistanceSensor distback;
    double driveMode = 0;
    double turn = 0;
    double boom = 0;

    int RF_deg = 0;
    int RB_deg = 0;
    boolean Right = false;
    boolean forward = false;
    double reach = 0;
    double Servo_pos = 1;
    double boom_arm = 0;
    double FL_Power = 0;
    double FR_Power = 0;
    double BL_Power = 0;
    double BR_Power = 0;
    double Left_deg_double = 0;
    double LeftPower;
    double RightPower;
    double FrontPower;
    double BackPower;
    double heading = 0;
    double RedLineColor = 800;
    double BlueLineColor = 800;
    double CorrectionFactor = 0.03;
    //SkyStonePosition is the location of the skystone as determined by the FindSkyStone function
    int SkyStonePosition = 0;
    //SkyStoneOffset is the offset of the robot from the skystone as determined by the FindSkyStone function
    double SkyStoneOffset = 0;
    DigitalChannel l_limit;
    DigitalChannel u_limit;
    DigitalChannel f_limit;
    DigitalChannel c_limit;
    DigitalChannel arm_u_limit;
    DigitalChannel arm_l_limit;
    boolean Red_alliance = false;
    boolean FirstGyroRun = true;
    double rampmotorpower;
    boolean QuestionAnswered = false;
    boolean distR;
    boolean WafflesRun = false;
    boolean LineRun = false;
    boolean LoopsRun = false;
    boolean ConquorRun = false;
    boolean up;
    double dst_heading;
    public ArrayList allTrackables;
    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    // Variables for Camera / Vuforia
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false  ;

    private static final String VUFORIA_KEY = "Ae6MFyH/////AAABmf06rZVcN0VqtDgoLd1KtAcowILYnLgT+SXkGwocAFEpSEmZiGI51vbPAL/QfAgSIhIpPrAK3Fl+vReEgcd1kU5az1pYTI01VqIV1+3ELPbCEcnNMdw3Rs7L8tMMsyfY2nebMlSFfgn6rkfJnnoQEnr4gCGHB1K/7VVZsFg7AlG4SPJ1bORRlhkFf0xIP28Tvr80YnC06hnsL2AQgOJtGrPv7HUO04hWxe9jLMdwHhmnBu/FZfovI9A6EjrzB72joGNTzuLA5R2bBGKW6AsrkKcgw1y50EFOqanZ19gWBX7bc1OExeaNeNIMaGzbMV8jwVahVndqS4EiLc9FuudY21tw4b4jupvhSYUiSGBMtLmh\";\n";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;

    //-------------------------------------------------------------------------------------------------
    public void GyroDriveBase(boolean RampUp, int Course, double motorpower, boolean strafe, boolean right)
    {
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
            FL_drive.setPower(LeftPower);
            BL_drive.setPower(LeftPower);
            FR_drive.setPower(-RightPower);
            BR_drive.setPower(-RightPower);
        }
        else if (strafe) {
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
            } else if (dst_heading > 0 && right)  {
                    BackPower = (motorpower - (dst_heading * CorrectionFactor));
                    FrontPower = motorpower;
            } else {
                FrontPower = motorpower;
                BackPower = motorpower;
            }
            if (right) {
                FL_drive.setPower(FrontPower);
                BL_drive.setPower(-BackPower);
                FR_drive.setPower(FrontPower);
                BR_drive.setPower(-BackPower);
            }
            else {
                FL_drive.setPower(-FrontPower);
                BL_drive.setPower(BackPower);
                FR_drive.setPower(-FrontPower);
                BR_drive.setPower(BackPower);
            }
        }

    }

    //This function is used to consistently turn off powers to all of the motors and reset the encoders after every function.
    public void resetmotors() {
        BR_drive.setPower(0);
        BL_drive.setPower(0);
        FR_drive.setPower(0);
        FL_drive.setPower(0);
        The_Boom.setPower(0);
        The_Boom2.setPower(0);
        Reach.setPower(0);
        Boom_Arm.setPower(0);

        FL_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FL_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //-------------------------------------------------------------------------------------------------
    public void drivedeg(int drivedegrees, double motorpower, int drivetime, boolean StopMotors, int Course, boolean RampUp) {
        FL_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FirstGyroRun = true;

        //The negative for the right motor to move in the correct direction

        //If we're drive backwards, we're looking at negative degrees and need to switch the operator in the while loop
        if (drivedegrees < 0) {
            while ((FL_drive.getCurrentPosition() >=  drivedegrees) && (BL_drive.getCurrentPosition() >=  drivedegrees)&& (BR_drive.getCurrentPosition() <=  -drivedegrees) && (FR_drive.getCurrentPosition() <=  -drivedegrees) && (opModeIsActive())) {
           GyroDriveBase(RampUp,Course,motorpower, false, false);
            }}
        else {
            while (((FL_drive.getCurrentPosition() <=  drivedegrees) && (BL_drive.getCurrentPosition() <=  drivedegrees)&& (BR_drive.getCurrentPosition() >=  -drivedegrees) && (FR_drive.getCurrentPosition() >=  -drivedegrees)) && (opModeIsActive())) {
                GyroDriveBase(RampUp,Course,motorpower, false, false);

            }}

    telemetry.addData("Encoders RF RB LF LB", "%7d :%7d :%7d :%7d",
            FR_drive.getCurrentPosition(),
            BR_drive.getCurrentPosition(),
            FL_drive.getCurrentPosition(),
            BL_drive.getCurrentPosition());
    telemetry.update();


//turn off motors
        if (StopMotors)
        resetmotors();
        //pause to give motors a rest NO GRINDING GEARS
        sleep(drivetime);
        FirstGyroRun=true;
    }
    //-------------------------------------------------------------------------------------------------

    public void drivedist(int distance, double motorpower, int drivetime, boolean FrontSensor, int Course, boolean RampUp) {
        if (FrontSensor) {
            // Check front distance sensor to be sure we're not already too close
            if (distfront.getDistance(DistanceUnit.INCH)  > distance)
            {
                // Leave motors powered until we close the distance to less than we passed in
                while ((distfront.getDistance(DistanceUnit.INCH)  > distance) && (opModeIsActive())) {
                    GyroDriveBase(RampUp,Course,motorpower, false, false);
                }
            }
            else {

                // Leave motors powered until we close the distance to less than we passed in
                while ((distfront.getDistance(DistanceUnit.INCH)  < distance) && (opModeIsActive())) {
                    GyroDriveBase(RampUp,Course,-motorpower, false, false);
                }
            }


        }
        //else is for using back sensor because the boolean frontsensor is false.
        else {
            // Check back distance sensor to be sure we're not already too close
            if (distback.getDistance(DistanceUnit.INCH)  > distance)
            {
                // Leave motors powered until we close the distance to less than we passed in
                while ((distback.getDistance(DistanceUnit.INCH)  > distance) && (opModeIsActive())) {
                    GyroDriveBase(RampUp,Course,-motorpower, false, false);
                }
            }
            else {
                // Leave motors powered until we expand the distance to more than we passed in
                while ((distback.getDistance(DistanceUnit.INCH) < distance) && (opModeIsActive())) {
                    GyroDriveBase(RampUp,Course,motorpower, false, false);
                }
            }
        }
        //turn off motors
        resetmotors();
        //pause to give motors a rest NO GRINDING GEARS
        FirstGyroRun = true;
        sleep(drivetime);
    }
    public void SFColor (boolean Red, double motorpower, boolean Right, boolean StopOnLine, int Course, boolean RampUp){
        if (Red) {
            //This will strafe to the red line
            while ((colorSensor.red() < RedLineColor) && (opModeIsActive())) {
            GyroDriveBase(RampUp,Course,motorpower,true,Right);
            }
      }
        else {
        //This will strafe to the blue line
        while ((colorSensor.blue() < BlueLineColor) && (opModeIsActive())) {
            GyroDriveBase(RampUp,Course,motorpower,true,Right);
        }
       }

//turn off motors
        if (StopOnLine)
        resetmotors();
        FirstGyroRun = true;
    }

    public void DriveColor (boolean Red, double motorpower, boolean StopOnLine, boolean RampUp, int Course){
        if (Red) {
            //This will drive backwards to the Red Line
            while ((colorSensor.red() < RedLineColor) && (opModeIsActive())) {
                GyroDriveBase(RampUp, Course, motorpower, false, false);
            }
        }
        // If Blue
        else {
            //This will drive to the Red Line
            while ((colorSensor.blue() < BlueLineColor) && (opModeIsActive())) {
                GyroDriveBase(RampUp, Course, motorpower, false, false);
            }
        }

//turn off motors if the boolean StopOnLine is true
        if (StopOnLine)
            resetmotors();
        FirstGyroRun = true;
    }

    public void SFdist (int dist, double motorpower,boolean distR, boolean StopMotors, boolean Right, int Course, boolean RampUp){
        //If we want to use the LEFT distance sensor, then it will go through each of the following while loops to determine which direction to move.
    if (distR == false) {
    //This will strafe left using the left distance sensor moving TOWARD the object on the left
    if (!Right) while ((distleft.getDistance(DistanceUnit.INCH) >= dist) && (opModeIsActive())) {
        GyroDriveBase(RampUp,Course,motorpower,true,Right);
    }
    //This will strafe right using the left distance sensor moving AWAY from the object on the left
    else while ((distleft.getDistance(DistanceUnit.INCH) <= dist) && (opModeIsActive())) {
        GyroDriveBase(RampUp,Course,motorpower,true,Right);
    }

}
//Use the right Distance Sensor
                if (distR == true) {

                   if (Right) while ((distright.getDistance(DistanceUnit.INCH) >= dist) && (opModeIsActive())) {
                       // Strafe Right
                        GyroDriveBase(RampUp,Course,motorpower,true,Right);
                    }
                    else while ((distright.getDistance(DistanceUnit.INCH) <= dist) && (opModeIsActive())) {
                        //Strafe Left
                        GyroDriveBase(RampUp,Course,motorpower,true,Right);
                    }
                }

//turn off motors
        if (StopMotors)
        resetmotors();
        FirstGyroRun = true;
    }

    //-------------------------------------------------------------------------------------------------
//Strafe using the motor encoders
    public void SF(int deg, double motorpower, int drivetime, boolean Right, boolean StopMotors, int Course, boolean RampUp) {
        FL_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//Check to see if we're strafing right or left then run until encoder values are met then exit.
        if (Right)
            while (((FL_drive.getCurrentPosition() <=  deg) && (FR_drive.getCurrentPosition() <=  deg)&& (BL_drive.getCurrentPosition() >=  -deg) && (BR_drive.getCurrentPosition() >=  -deg)) && (opModeIsActive())) {
                GyroDriveBase(RampUp,Course,motorpower,true,Right);
            }
        else
            while (((BL_drive.getCurrentPosition() <=  deg) && (BR_drive.getCurrentPosition() <=  deg)&& (FR_drive.getCurrentPosition() >=  -deg) && (FL_drive.getCurrentPosition() >=  -deg)) && (opModeIsActive())) {
                GyroDriveBase(RampUp,Course,motorpower,true,Right);
            }


//turn off motors
        if(StopMotors)
        resetmotors();
        sleep(drivetime);
        FirstGyroRun = true;
    }
    //-------------------------------------------------------------------------------------------------

    public void SpinGyro (double dest_heading, double motorpower, long drivetime, boolean Right, boolean ResetMotors) {
        if (Red_alliance == false) dest_heading = dest_heading * -1;
        long starttime = System.currentTimeMillis();
        checkOrientation();
        //If Right boolean is true it will spin right
        if (Right) {
                // Turn on the motors
                FL_drive.setPower(motorpower);
                BL_drive.setPower(motorpower);
                FR_drive.setPower(motorpower);
                BR_drive.setPower(motorpower);
                //Wait for heading to be achieved - if heading can't be achieved, timeout and exit the loop
            while ((heading >= dest_heading) && (System.currentTimeMillis() <= starttime+drivetime) && opModeIsActive()) {
                checkOrientation();
            }
        }
        //If false it will spin left
        else {
                    // Turn on the motors
                    FL_drive.setPower(-motorpower);
                    BL_drive.setPower(-motorpower);
                    FR_drive.setPower(-motorpower);
                    BR_drive.setPower(-motorpower);
            while ((heading <= dest_heading) &&  (System.currentTimeMillis() <= starttime+drivetime) && (opModeIsActive())) {
                checkOrientation();
            }
            }

        //turn off motors if the boolean ResetMotors is true.
        if (ResetMotors)
        resetmotors();
        FirstGyroRun = true;
}
    //-------------------------------------------------------------------------------------------------


    public void MoveBoom(int deg,int motorPower,int time) {
        Boom_Arm.setTargetPosition(deg);

        Boom_Arm.setPower(motorPower);

        Boom_Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        sleep(time);

        Boom_Arm.setPower(0);

        Boom_Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
    //-------------------------------------------------------------------------------------------------
    public void MoveReach(int motorPower,int time) {
        Reach.setPower(motorPower);

        sleep(time);

        Reach.setPower(0);
    }
    //-------------------------------------------------------------------------------------------------
    public void chopsticks (boolean up){
        if (up == true){
            right_chop.setPosition(0.5);
            left_chop.setPosition(0.5);
        }
        if (up == false){
            right_chop.setPosition(1);
            left_chop.setPosition(0);
        }
    }


    public void FindSkyStone(ArrayList<VuforiaTrackable> allTrackables, boolean Right) {
        targetVisible = false;

//SkyStonePosition enters as 0, we want to run the while loop 3 times but on the 3rd time, we don't want to use the camera, we just want to increment the SkyStonePosition to 3.
        SkyStonePosition = 0;
        while ((!targetVisible) && (SkyStonePosition <= 2)  && opModeIsActive()) {

            SkyStonePosition = SkyStonePosition + 1;
            if (SkyStonePosition <= 2) {
                //sleep(3000);
                int loopcount =0;
                int highloop = 200;
                while (loopcount < highloop &&  !targetVisible && opModeIsActive()) {
                    loopcount = loopcount +1;
                    for (VuforiaTrackable trackable : allTrackables) {
                    if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
//                        telemetry.addData("Visible Target, SkyStonePosition", trackable.getName(), SkyStonePosition);
                        telemetry.addData("VISIBLE!:", "%7d", SkyStonePosition);
                        telemetry.update();
                        targetVisible = true;
                        // getUpdatedRobotLocation() will return null if no new information is available since
                        // the last time that call was made, or if the trackable is not currently visible.
                        OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                        if (robotLocationTransform != null) {
                            lastLocation = robotLocationTransform;
                        }
                        break;
                    }
                }
            //Strafe one block width (8") if we don't see the block yet - and don't strafe if this is the 3rd run/block
            if ((!targetVisible) && (SkyStonePosition == 1) && (loopcount == highloop) && opModeIsActive()) {
                //Strafe to look at the next SkyStone only if this is the first time through and we didn't see the stone
                SF(505, .35, 0, Right, true, 0, false);
                //Pause for a moment to let the motors settle and the camera to find the SkyStone
                telemetry.addLine();
                telemetry.addData("NOPE:", "%7d", SkyStonePosition);
                telemetry.update();
                //sleep(250);
            }}

            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible) {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                SkyStoneOffset = translation.get(1);
                telemetry.addLine();
                telemetry.addData("VISIBLE", "%7d", SkyStonePosition);
                telemetry.update();
                //telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                //        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                // express the rotation of the robot in degrees.
                //Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                //telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
            } else {
               // telemetry.addData("Visible Target, SkystonePosition", "none", SkyStonePosition);
                telemetry.addData("NOPE", "%7d", SkyStonePosition);
                telemetry.update();

            }
            }
            //Turn off the light ont he camera
            CameraDevice.getInstance().setFlashTorchMode(false);
        }
    }
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

    public void FibbyConquorRed(ArrayList<VuforiaTrackable> allTrackables, boolean Red_Alliance) {

        boolean RightTurn=true;
        boolean LeftTurn=false;
        boolean distR=true;
        boolean distL=false;
        if (!Red_alliance) {
            RightTurn=false;
            LeftTurn=true;
            distR=false;
            distL=true;
        }

//strafe degrees = 62.35 per inch we think
        //Turn on the camera light

        //raise boom
        //Start driving while getting moving extremities out of the way
         FL_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
         BL_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
         FR_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
         BR_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Reach.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Boom_Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Boom_Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Reach.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         //turn motors on
        Drivetime(0.3,100,false,0,true);

        Boom_Arm.setTargetPosition(2000);
        Boom_Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Boom_Arm.setPower(1);
        Reach.setTargetPosition(-2000);
        Reach.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Reach.setPower(1);
        //Open mouth
        servo.setPosition(0);
        sleep(500);
        // don't use front distance sensor until the arm is out of the way changed from 16
        drivedist(16, .3, 100, true, 0, false);
        //Look for a skystone using vuforia
        FindSkyStone(allTrackables, LeftTurn);
        //lower boom arm after the camera has detected the skystone
        Boom_Arm.setTargetPosition(400);
        Boom_Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Boom_Arm.setPower(1);
        if (SkyStonePosition == 3)
        SF(505, .5, 0, LeftTurn, true, 0, false);
        else sleep(200);
       // targetsSkyStone.deactivate();
        //drive forward to collect the skystone
        drivedeg(500,0.4,0, true, 0,false);
        //close mouth to grab the first skystone
        servo.setPosition(0.5);
        sleep(50);
        //suck in reach to fit under bridge
        Reach.setTargetPosition(1700);
        Reach.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Reach.setPower(1);
        //raise boom arm to take weight off skystone
        Boom_Arm.setTargetPosition(700);
        Boom_Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Boom_Arm.setPower(1);
        sleep(50);
        //back up away from skystones
        drivedist(23,1,0,false, 0,true);
        SFColor(RightTurn,1,RightTurn,false, 0, true);
        //Start raising the boom arm after crossing under the bridge
        Boom_Arm.setTargetPosition(1750);
        Boom_Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Boom_Arm.setPower(0.7);
        //Strafe towards wall to line up with the foundation
        //Distance was 23" on 2-17-20 but changed to 17 because it started not grabbing the right place suddenly
        SFdist(23, 1,RightTurn, true, RightTurn, 0, false);
        //Drive toward the foundation
        drivedist(29,0.8,0,false, 0, true);
        //Release the krackin!
        servo.setPosition(0);
        //Slow down to not knock the foundation away
        drivedeg(200,0.3,0,true, 0, false);
        //grab foundation with chop chop sticks
        left_chop.setPosition(0);
        right_chop.setPosition(1);

        sleep(500);
        //backing up to wall using the rear distance sensor
        drivedist(11,1,100,false, 0, true);
        SFdist(24,1,RightTurn,true, LeftTurn,0,true);
        SpinGyro(-90, 1, 3000, RightTurn, true);

        //release the foundashen
        left_chop.setPosition(0.5);
        right_chop.setPosition(0.5);

        //Give the chopsticks a moment to raise
        SFdist(19, 1,RightTurn, true, RightTurn, 90,true);
        Drivetime(-0.5,300,false, 90, false);
        //True the robot up before backing up.
        //checkOrientation();
        //if (heading < -91)
        //SpinGyro(-90, 0.1, 1000, LeftTurn, true);
        //else if (heading > -89)
        //SpinGyro(-90, 0.1, 1000, RightTurn, true);
        // strafe away from wall to avoid our alliance partner
        SFdist(22,1,RightTurn,true, LeftTurn, 90, true);
        //bring arm down to go under bridge and grab the next skystone
        Boom_Arm.setTargetPosition(400);
        Boom_Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Boom_Arm.setPower(1);
       //go under the bridge
        DriveColor(RightTurn,-1,false,false,90);
        //Drivetime(-1,1000, false);
        Reach.setTargetPosition(-2000);
        Reach.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Reach.setPower(1);
        //Back up to allign with the next SkyStone
        drivedist(36,1,100,false, 90, false);

        //Start spinning fast then slow down to be more acurate.
        SpinGyro(-9,0.5,2000,LeftTurn,false);

        if (SkyStonePosition == 1) {
            //Grab stone position #4
            SFdist(20,0.75,LeftTurn,true, LeftTurn, 0, true); }
        else {
            //Grab stone position #5
            SFdist(10,0.75,LeftTurn,true,LeftTurn, 0, true); }
// Drive to block
        drivedist(26, .4, 0, false, 0, true);

        servo.setPosition(0.5);
        //suck in reach
        //Reach.setTargetPosition(-1500);
        //Reach.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //Reach.setPower(1);
        //raise boom arm to take weight off skystone
        Boom_Arm.setTargetPosition(700);
        Boom_Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Boom_Arm.setPower(1);
        //back up away from skystones
        //drivedeg(-620,-620,-620,-620,1,100,true);
        drivedist(26,1,0,false, 0, true);
        SpinGyro(-82,0.5,2000,RightTurn,false);
        SFdist(25,1,true,true,false, 90, true);
        Reach.setTargetPosition(-6000);
        Reach.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Reach.setPower(1);
        DriveColor(RightTurn,1,false,true, 90);
        Boom_Arm.setTargetPosition(2000);
        Boom_Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Boom_Arm.setPower(1);
        drivedeg(2000,0.9,0,true, 95, false);
         //Release the krackin!
        servo.setPosition(0);
        DriveColor(RightTurn,-0.9,true,true,  90);
        //(Steve releasing the party in a box) CELEBRATE!!!! Please!
    }


    public void WaffleV2 (){
        long startruntime = System.currentTimeMillis();
        boolean RightTurn=true;
        boolean LeftTurn=false;
        boolean distR=true;
        boolean distL=false;
        if (!Red_alliance) {
            RightTurn=false;
            LeftTurn=true;
            distR=false;
            distL=true;
        }

        Boom_Arm.setTargetPosition(2000);
        Boom_Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Boom_Arm.setPower(1);

//drive away from wall to avoid sliding along the wall
        drivedeg(100,0.5,0,true,0,true);
        //Strafe towards wall to line up with the foundation
        //Distance was 23" on 2-17-20 but changed to 17 because it started not grabbing the right place suddenly
        if (Red_alliance)
        SFdist(21, 0.75,RightTurn, true, RightTurn, 0, false);
        else
            SFdist(19, 0.75,RightTurn, true, RightTurn, 0, false);
        //Drive toward the foundation
        drivedist(28,0.3,0,false, 0, true);
        //Slow down to not knock the foundation away
        drivedeg(300,0.25,0,true, 0, false);
        //grab foundation with chop chop sticks
        left_chop.setPosition(0);
        right_chop.setPosition(1);
        //stop for a half second to allow the chops to raise
        sleep(500);
        //backing up to wall using the rear distance sensor
        drivedist(11,1,100,false, 0, true);
        //strafe away from the corner to the edge of the foundation
        SFdist(24,1,RightTurn,true, LeftTurn,0,true);
        //SPIN THE WAFFLE CRONK
        SpinGyro(-90, 1, 3000, RightTurn, true);

        //release the foundation
        left_chop.setPosition(0.5);
        right_chop.setPosition(0.5);

        //Give the chopsticks a moment to raise while we center on the foundation
        SFdist(15, 1,RightTurn, true, RightTurn, 90,true);
        //push waffle into the back wall
        Drivetime(0.5,750,true, 90, false);
        //back away from the foundation
        drivedeg(-100,-0.5,0,true,90,false);
        //strafe towards the wall to line up with our spot under the bridge
        if (Red_alliance)
        SFdist(3,0.3,RightTurn,true,RightTurn,90,false);
        else
            SFdist(2,0.3,RightTurn,true,RightTurn,90,false);
        // wait till the last 3 seconds till we backup to park under the bridge
        resetmotors();
        //
        Boom_Arm.setTargetPosition(400);
        Boom_Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Boom_Arm.setPower(1);
        Reach.setTargetPosition(-3500);
        Reach.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Reach.setPower(1);
        while ((System.currentTimeMillis() <= startruntime+27000) && (opModeIsActive())) {}
        DriveColor(RightTurn,-0.5,true,false,90);
        //
    }
    //===============================================================================================

public void Loops (ArrayList<VuforiaTrackable> allTrackables, boolean Red_Alliance){

    boolean RightTurn=true;
    boolean LeftTurn=false;
    boolean distR=true;
    boolean distL=false;
    if (!Red_alliance) {
        RightTurn=false;
        LeftTurn=true;
        distR=false;
        distL=true;
    }

//strafe degrees = 62.35 per inch we think
    //raise boom
    //Start driving while getting moving extremities out of the way
    FL_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    BL_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    FR_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    BR_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    Reach.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    Boom_Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    Boom_Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    Reach.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    CameraDevice.getInstance().setFlashTorchMode(true);
    //turn motors on
    Drivetime(0.3,100,false,0,true);

    Boom_Arm.setTargetPosition(1700);
    Boom_Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    Boom_Arm.setPower(1);
    Reach.setTargetPosition(-2000);
    Reach.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    Reach.setPower(1);
    //Open mouth
    servo.setPosition(0);
    sleep(500);
    // don't use front distance sensor until the arm is out of the way changed from 16
    drivedist(14, .3, 100, true, 0, false);
    //Look for a skystone using vuforia
    FindSkyStone(allTrackables, LeftTurn);
    //lower boom arm after the camera has detected the skystone
    Boom_Arm.setTargetPosition(400);
    Boom_Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    Boom_Arm.setPower(1);
    if (SkyStonePosition == 3)
        SF(505, .5, 0, LeftTurn, false, 0, false);
    else sleep(200);
    // targetsSkyStone.deactivate();
    //drive forward to collect the skystone
    drivedeg(450,0.2,0, true, 0,true);
    //close mouth to grab the first skystone
    servo.setPosition(0.5);
    sleep(200);
    //raise boom arm to take weight off skystone
    //Boom_Arm.setTargetPosition(700);
    //Boom_Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    //Boom_Arm.setPower(1);
    //back up away from skystones
    drivedist(23,1,0,false, 0,true);
    if (SkyStonePosition == 1)
    SpinGyro(-82,0.5,2000,RightTurn,false);
    else
    Drivetime(0.75,1000,false,90,false);
    //The right distance sensor is reading 2" more than the left sensor so for the strafes based on the wall we need to compensate
    if (Red_Alliance)
    SFdist(24,0.4,RightTurn,true,LeftTurn, 90, true);
    else //blue alliance
    SFdist(22,0.4,RightTurn,true,LeftTurn, 90, true);
    // turn to red line then stop on red line
    DriveColor(RightTurn,0.75, false ,true,90);
    drivedeg(800,0.75,0,false, 90,false);
    drivedeg(500,0.5,100,true,45,false);
    //drop off skystone
    servo.setPosition(0);
    //drop chopsticks to drop capstone
    left_chop.setPosition(0);
    right_chop.setPosition(1);
    drivedeg(-500,-0.5,0,false,90,false);
    DriveColor(RightTurn,-0.75,false,false,90);
    //put them in postion to grab waffle
    left_chop.setPosition(0.5);
    right_chop.setPosition(0.5);
    // Close the servo if position is 3 to flick the 5th skystone out of the way-
    if (SkyStonePosition == 3)
    servo.setPosition(1);
    //Back up to allign with the next SkyStone
    drivedist(36,1,100,false, 90, false);

    //Start spinning fast then slow down to be more acurate.
    SpinGyro(-9,0.5,2000,LeftTurn,false);
    //if (heading > 0)
    //SpinGyro(0,0.2,1000,RightTurn,true);
    drivedist(19,0.5,0,false,0,true);

    if (SkyStonePosition == 1) {
        //Grab stone position #4
        if (Red_Alliance)
        SFdist(16,0.75,LeftTurn,true, LeftTurn, 0, true);
        else
            SFdist(17,0.75,LeftTurn,true, LeftTurn, 0, true);
    }
    else if (SkyStonePosition == 2) {
        //Grab stone position #5
        if (Red_Alliance)
        SFdist(8,0.75,LeftTurn,true,LeftTurn, 0, true);
    else
        SFdist(11,0.75,LeftTurn,true,LeftTurn, 0, true);
    }
    // Grab stone position #6
    else {
        //move reach out
        Reach.setTargetPosition(-6000);
        Reach.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Reach.setPower(1);
        //strafe to position #6
        if (Red_Alliance)
        SFdist(8, 0.75, LeftTurn, true, LeftTurn, 0, true);
        else
            SFdist(10, 0.75, LeftTurn, true, LeftTurn, 0, true);
        //move it out of the way
        drivedist(24, 0.3, 0, false, 0, true);
        // flick the block out of the way
        servo.setPosition(0);
        //pull reach back in
        Reach.setTargetPosition(-1300);
        Reach.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Reach.setPower(1);
        // wait for line before to complete before moving on
        sleep(1250);
        //strafe to wall
        SFtime(750, 0.5, LeftTurn, false, 0, false);
        Drivetime(0.3, 250, true, 0, false);
        //not quite clamp on block
        servo.setPosition(0.4);
        sleep(500);
        //Spin a bit to get the block
        Drivetime(0.3, 1000, true, -45, false);
        //fully clamp block
        servo.setPosition(0.5);
        //back awawy from blocks
        drivedist(23, 0.4, 0, false, 0, true);
        //back up away from skystones
        //drivedist(23,1,0,false, 0,true);
        SFdist(4, 0.5, LeftTurn, false, RightTurn, 0, false);
    }
    //Run the code to grab the block and back up only for positions 1 or 2, position 3 runs differently
    if (SkyStonePosition == 1 || SkyStonePosition == 2) {
        //Drive forward to skystone
        drivedist(27, .25, 0, false, 0, true);
        //Close gripper
        servo.setPosition(0.5);
        sleep(200);
        //back up away from skystones
        drivedist(26, 1, 0, false, 0, true);
    }
    //Turn toward bridge
      //  SpinGyro(-82,0.5,2000,RightTurn,false);
    Drivetime(0.75, 1200,false, 90, true);
    //Distance sensors are measuring different on each side of the robot, this adjusts their measurements
    if (Red_Alliance)
        // Strafe away from wall to align to bridge
        SFdist(24,0.4,RightTurn,false,LeftTurn, 90, false);
    else
        SFdist(23,0.4,RightTurn,false,LeftTurn, 90, false);
        // drive to red line
        DriveColor(RightTurn,0.7, false ,false,90);
        //keep driving past bridge/line to get the stone all of the way across
        drivedeg(800,0.6,0,true, 90,false);
        //Open gripper to release stone
        servo.setPosition(0);
        Boom_Arm.setTargetPosition(400);
        Boom_Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Boom_Arm.setPower(1);
        Reach.setTargetPosition(-3500);
        Reach.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Reach.setPower(1);
        //Drive back to red line to park
        DriveColor(RightTurn,-0.5,true,false,90);
        servo.setPosition(1);

    //(Steve releasing the party in a box) CELEBRATE!!!!
}

    public void test(){

        resetmotors();


    }
    private void lineLeft() {
        Reach.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Boom_Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Boom_Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Reach.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //take a 26 second nap
        sleep(26000);



        //the robot reaches over line
        Boom_Arm.setTargetPosition(400);
        Boom_Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Boom_Arm.setPower(1);
        Reach.setTargetPosition(-3500);
        Reach.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Reach.setPower(1);
        //Reach.setTargetPosition(-1500);
        //Reach.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //Reach.setPower(1);
        //MoveBoom(1500,1,3000);

        sleep (3000);
    }

    private void checkOrientation() {
// read the orientation of the robot
        angles = this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        this.imu.getPosition();
// and save the heading
        heading = angles.firstAngle;
    }
    public void runOpMode() {


            //Color sensor prep
            // hsvValues is an array that will hold the hue, saturation, and value information.
            float hsvValues[] = {0F,0F,0F};
            // values is a reference to the hsvValues array.
            final float values[] = hsvValues;
            colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");
            colorSensor.enableLed(true);

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
// Prepare Camera / Vuforia
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
            parameters.vuforiaLicenseKey = VUFORIA_KEY;
            parameters.cameraDirection   = CAMERA_CHOICE;
            vuforia = ClassFactory.getInstance().createVuforia(parameters);

            VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");
            VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
            stoneTarget.setName("Stone Target");

            ArrayList<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
            allTrackables.addAll(targetsSkyStone);

            stoneTarget.setLocation(OpenGLMatrix
                    .translation(0, 0, stoneZ)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
            if (CAMERA_CHOICE == BACK) {
                phoneYRotate = -90;
            } else {
                phoneYRotate = 90;
            }
            final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
            final float CAMERA_VERTICAL_DISPLACEMENT = 4.5f * mmPerInch;   // eg: Camera is 8 Inches above ground
            final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

            OpenGLMatrix robotFromCamera = OpenGLMatrix
                    .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));
            for (VuforiaTrackable trackable : allTrackables) {
                ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
            }

        //Mapping motor to variable names
        BR_drive = hardwareMap.dcMotor.get("BR_drive");
        BL_drive = hardwareMap.dcMotor.get("BL_drive");
        FR_drive = hardwareMap.dcMotor.get("FR_drive");
        FL_drive = hardwareMap.dcMotor.get("FL_drive");
        The_Boom = hardwareMap.dcMotor.get("The_Boom");
        The_Boom2 = hardwareMap.dcMotor.get("The_Boom2");
        Reach = hardwareMap.dcMotor.get ("Reach");
        Boom_Arm= hardwareMap.dcMotor.get ("Boom_Arm");
        servo = hardwareMap.servo.get("servo0");
        left_chop = hardwareMap.servo.get("left_chop");
        right_chop = hardwareMap.servo.get("right_chop");

        //LED LIGHTS


        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

        //Resetting motor encoders
       resetmotors();

        //Setting brake to stop motor quickly to not destroy limit switches
        Reach.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        l_limit = hardwareMap.get(DigitalChannel.class, "l_limit");
        l_limit.setMode(DigitalChannel.Mode.INPUT);

        // upper Limit switch definition for Boom
        u_limit = hardwareMap.get(DigitalChannel.class, "u_limit");
        u_limit.setMode(DigitalChannel.Mode.INPUT);

        c_limit = hardwareMap.get(DigitalChannel.class, "c_limit");
        c_limit.setMode(DigitalChannel.Mode.INPUT);

        f_limit = hardwareMap.get(DigitalChannel.class, "f_limit");
        f_limit.setMode(DigitalChannel.Mode.INPUT);

        arm_u_limit = hardwareMap.get(DigitalChannel.class, "arm_u_limit");
        arm_u_limit.setMode(DigitalChannel.Mode.INPUT);

        arm_l_limit = hardwareMap.get(DigitalChannel.class, "arm_l_limit");
        arm_l_limit.setMode(DigitalChannel.Mode.INPUT);

        distleft = hardwareMap.get(DistanceSensor.class, "distleft");

        distright = hardwareMap.get(DistanceSensor.class, "distright");

        distfront = hardwareMap.get(DistanceSensor.class, "distfront");

        distback = hardwareMap.get(DistanceSensor.class, "distback");

        // Robot, squeeze into your box! the following code resets the robot to starting position
        //closing the servo
        Servo_pos = 1;
        servo.setPosition(Servo_pos);
            //opening the chops to ready them for foundation.
            left_chop.setPosition(0);
            right_chop.setPosition(1);

        // resetting the arm to fit in the box
        while(arm_l_limit.getState() == false)
         {
             Boom_Arm.setPower(-0.6);
        }
        Boom_Arm.setPower(0);
        //resetting the reach
        while (c_limit.getState() == false) {
            Reach.setPower(0.6);
        }
        Reach.setPower(0);
            Boom_Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            //opening the chops to ready them for foundation.
            left_chop.setPosition(0.5);
            right_chop.setPosition(0.5);


        //Are we on the Red or Blue Alliance?
            telemetry.addLine("ALLIANCE: Red or Blue?");
            telemetry.update();
         while (QuestionAnswered == false) {
             if (gamepad1.b) {

                 Red_alliance = true;
                 pattern = RevBlinkinLedDriver.BlinkinPattern.LARSON_SCANNER_RED;
                 blinkinLedDriver.setPattern(pattern);
                 QuestionAnswered = true;

             }
             else if (gamepad1.x){
                 Red_alliance = false;
                 pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
                 blinkinLedDriver.setPattern(pattern);
                 QuestionAnswered = true;

             }
         }


         //selecting auto
         QuestionAnswered = false;
            telemetry.addLine("Y for Conquor - A for line - X for Loops - B for Waffles?");
            telemetry.update();
            sleep (1000);
            while (QuestionAnswered == false) {
                if (gamepad1.y) {
                    //Conquor!
                    telemetry.addLine("Conquor the world!");
                    ConquorRun = true;
                    QuestionAnswered = true;
                }
                if (gamepad1.a){
                    //Strafe to line, our alliance partner must be amazing! or something broke :(
                    telemetry.addLine("Stafe to line");
                    LineRun = true;
                    QuestionAnswered = true;
                }
                if (gamepad1.x){
                    //Strafe to line, our alliance partner must be amazing! or something broke :(
                    telemetry.addLine("Loops");
                    LoopsRun = true;
                    QuestionAnswered = true;
                }
                if (gamepad1.b){
                    //Strafe to line, our alliance partner must be amazing! or something broke :(
                    telemetry.addLine("Waffles for breakfast");
                    WafflesRun = true;
                    QuestionAnswered = true;
                }
                telemetry.update();
            }


            //Start camera
            targetsSkyStone.activate();
        AutoTransitioner.transitionOnStop(this, "FibbyTeleOp20");


        targetsSkyStone.activate();
        while (!isStarted()) {
            //Will run during INIT
            /*telemetry.addData("Encoders",  "Starting at %7d :%7d :%7d :%7d",
                    FR_drive.getCurrentPosition(),
                    FL_drive.getCurrentPosition(),
                    BR_drive.getCurrentPosition(),
                    BL_drive.getCurrentPosition());
            telemetry.update();*/

        }
       // Red_alliance=true;
       // WaffleV2();
       // Loops(allTrackables, Red_alliance);

      // FibbyConquorRed(allTrackables, Red_alliance);
        //DriveColor(true,0.5,true, true);
        //Code beneath will start
        //Start autonomous period - 30 seconds to conquer the world!

        if (LoopsRun) Loops(allTrackables, Red_alliance);
       else if (WafflesRun) WaffleV2();
       else if (LineRun) lineLeft();
      else if (ConquorRun) FibbyConquorRed(allTrackables, Red_alliance);

        //turn off motors
        resetmotors();
          }

}