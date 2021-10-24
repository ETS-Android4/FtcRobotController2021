
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

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

import java.util.List;

@Autonomous(name="Aaaaahhhhh", group="Autonomous1")
@Disabled
public class Aaaaahhhhh extends LinearOpMode {

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
    int LF_deg = 0;
    int LB_deg = 0;
    int RF_deg = 0;
    int RB_deg = 0;
    boolean Right = false;
    double reach = 0;
    double Servo_pos = 1;
    double boom_arm = 0;
    double FL_Power = 0;
    double FR_Power = 0;
    double BL_Power = 0;
    double BR_Power = 0;
    double Left_deg_double = 0;
    double heading = 0;
    DigitalChannel l_limit;
    DigitalChannel u_limit;
    DigitalChannel f_limit;
    DigitalChannel c_limit;
    DigitalChannel arm_u_limit;
    DigitalChannel arm_l_limit;
    boolean Red_alliance =false;
    boolean QuestionAnswered = false;
    boolean distR;
    boolean AutoL;
    boolean up;
    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;
    //-------------------------------------------------------------------------------------------------
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
    public void drivedeg(int LF_deg, int RF_deg, int LB_deg, int RB_deg, double motorpower, int drivetime) {
        FL_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Left_deg_double = LF_deg* 0.775;
        //LF_deg = (int)Left_deg_double;
        //LB_deg = (int)Left_deg_double;
        //The negative for the right motor to move in the correct direction
        FL_drive.setTargetPosition(LF_deg);
        BL_drive.setTargetPosition(LB_deg);
        FR_drive.setTargetPosition(-RF_deg);
        BR_drive.setTargetPosition(-RB_deg);
        // tell the robot where to go
        FL_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // actually make the robot go there
        FL_drive.setPower(motorpower);
        BL_drive.setPower(motorpower);
        FR_drive.setPower(motorpower);
        BR_drive.setPower(motorpower);
        sleep(drivetime);
        telemetry.addData("Encoders RF RB LF LB",  "%7d :%7d :%7d :%7d",
                FR_drive.getCurrentPosition(),
                BR_drive.getCurrentPosition(),
                FL_drive.getCurrentPosition(),
                BL_drive.getCurrentPosition());

        telemetry.update();

//turn off motors
resetmotors();

    }
    //-------------------------------------------------------------------------------------------------
    public void SFdist (int dist, double motorpower,boolean distR, boolean Right){

        //If we want to use the LEFT distance sensor, then it will go through each of the following while loops to determine which direction to move.
if (distR == false) {
    //This will strafe left
    while ((distleft.getDistance(DistanceUnit.INCH) >= dist) && (Right == false) && (opModeIsActive())) {
        FL_drive.setPower(-motorpower);
        BL_drive.setPower(motorpower);
        FR_drive.setPower(-motorpower);
        BR_drive.setPower(motorpower);
    }
    //This will strafe right
    while ((distleft.getDistance(DistanceUnit.INCH) <= dist) && (Right == true) && (opModeIsActive())) {
        FL_drive.setPower(motorpower);
        BL_drive.setPower(-motorpower);
        FR_drive.setPower(motorpower);
        BR_drive.setPower(-motorpower);
    }

}
//Use the right Distance Sensor
                if (distR == true) {
                    while ((distright.getDistance(DistanceUnit.INCH) >= dist) && (Right == true) && (opModeIsActive())) {
                       // Strafe Right
                        FL_drive.setPower(motorpower);
                        BL_drive.setPower(-motorpower);
                        FR_drive.setPower(motorpower);
                        BR_drive.setPower(-motorpower);
                    }
                    while ((distright.getDistance(DistanceUnit.INCH) <= dist) && (Right == false) && (opModeIsActive())) {
                        //Strafe Left
                        FL_drive.setPower(-motorpower);
                        BL_drive.setPower(motorpower);
                        FR_drive.setPower(-motorpower);
                        BR_drive.setPower(motorpower);
                    }
                }

//turn off motors
        resetmotors();
    }

    //-------------------------------------------------------------------------------------------------

    public void SF(int deg, double motorpower, int drivetime, boolean Right) {

        //Left_deg_double = LF_deg* 0.775;
        //LF_deg = (int)Left_deg_double;
        //LB_deg = (int)Left_deg_double;

        if (Right == true && (opModeIsActive())) {
            //If true it will strafe right
            FL_drive.setTargetPosition(deg);
            BL_drive.setTargetPosition(-deg);
            FR_drive.setTargetPosition(deg);
            BR_drive.setTargetPosition(-deg);

        }

        else if (Right == false && (opModeIsActive())) {
            //If false it will strafe left
            FL_drive.setTargetPosition(-deg);
            BL_drive.setTargetPosition(deg);
            FR_drive.setTargetPosition(-deg);
            BR_drive.setTargetPosition(deg);

        }

        // tell the robot where to go
        FL_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // actually make the robot go there
        FL_drive.setPower(motorpower);
        BL_drive.setPower(motorpower);
        FR_drive.setPower(motorpower);
        BR_drive.setPower(motorpower);

        sleep(drivetime);
        telemetry.addData("Encoders RF RB LF LB",  "%7d :%7d :%7d :%7d",
                FR_drive.getCurrentPosition(),
                BR_drive.getCurrentPosition(),
                FL_drive.getCurrentPosition(),
                BL_drive.getCurrentPosition());

        telemetry.update();


//turn off motors
        resetmotors();
    }
    //-------------------------------------------------------------------------------------------------

    public void SpinGyro (double dest_heading, double motorpower, long drivetime, boolean Right) {
        if (Red_alliance == false) dest_heading = dest_heading * -1;
        long starttime = System.currentTimeMillis();
        if (Right == true) {
            //If true it will spin right
            // tell the robot where to go
            while ((heading >= dest_heading) && (System.currentTimeMillis() <= starttime+drivetime) && (opModeIsActive())) {
                checkOrientation();
                // actually make the robot go there
                FL_drive.setPower(motorpower);
                BL_drive.setPower(motorpower);
                FR_drive.setPower(motorpower);
                BR_drive.setPower(motorpower);
            }

        }

        else if (Right == false) {
                //If false it will spin left
                while ((heading <= dest_heading) &&  (System.currentTimeMillis() <= starttime+drivetime) && (opModeIsActive())) {
                    checkOrientation();
                    // actually make the robot go there
                    FL_drive.setPower(-motorpower);
                    BL_drive.setPower(-motorpower);
                    FR_drive.setPower(-motorpower);
                    BR_drive.setPower(-motorpower);


                }

            }

        //turn off motors
        resetmotors();
}
    //-------------------------------------------------------------------------------------------------

    public void Spin(int deg, double motorpower, int drivetime, boolean Right) {
        FL_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Left_deg_double = LF_deg* 0.775;
        //LF_deg = (int)Left_deg_double;
        //LB_deg = (int)Left_deg_double;

        if (Right == true) {
            //If true it will spin right
            FL_drive.setTargetPosition(deg);
            BL_drive.setTargetPosition(deg);
            FR_drive.setTargetPosition(deg);
            BR_drive.setTargetPosition(deg);

        }

        else if (Right == false){
            //If false it will spin left
            FL_drive.setTargetPosition(-deg);
            BL_drive.setTargetPosition(-deg);
            FR_drive.setTargetPosition(-deg);
            BR_drive.setTargetPosition(-deg);

        }

        // tell the robot where to go
        FL_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // actually make the robot go there
        FL_drive.setPower(motorpower);
        BL_drive.setPower(motorpower);
        FR_drive.setPower(motorpower);
        BR_drive.setPower(motorpower);
        sleep(drivetime);
        telemetry.addData("Encoders RF RB LF LB",  "%7d :%7d :%7d :%7d",
                FR_drive.getCurrentPosition(),
                BR_drive.getCurrentPosition(),
                FL_drive.getCurrentPosition(),
                BL_drive.getCurrentPosition());

        telemetry.update();


//turn off motors
        resetmotors();
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
    //-------------------------------------------------------------------------------------------------
    public void wafflesforbreakfast(boolean Red_alliance){
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

        //drive away from wall
        drivedeg(100,100,100,100,0.5,1000);
        //raise arm
        MoveBoom(5000,50,560);
        //move out reach
        MoveReach(-1,1000);
        //strafe right
        SFdist(20,0.5,distR,RightTurn);
        //SF(4700,0.5,4000,RightTurn);
        //move off wall
        //SF(700,0.5,1000,LeftTurn);

        //drive forward
        drivedeg(2350,2350,2350,2350,0.5,2500);
        //servo
        Servo_pos = 1;
        servo.setPosition(Servo_pos);
        //move arm down
        MoveBoom(-5500,50,600);
        //back up
        drivedeg(-2250,-2250,-2250,-2250,0.5,3000);
        //turn to the right
        //Spin(3500,0.5,3000,true);
        SpinGyro(-90,0.5,5000, RightTurn);
        //raise arm
        MoveBoom(5000,50,560);
        //True back up to -90 degrees as the first line will overshoot almost every time
        SpinGyro(-90,0.2,2000, LeftTurn);
        // back up
        drivedeg(-3200,-3200,-3200,-3200,0.5,2000);
        //strafe to wall
        SFdist(2,0.5,distR,RightTurn);
        //SF(300, 0.5, 500, RightTurn);
    }
   // ______________________________________________________________________________________________________________________

    public void loops(boolean Red_alliance){

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
        //lift arm
        MoveBoom(600,50,1000);
        //servo opens
        Servo_pos = 0;
        servo.setPosition(Servo_pos);
        //move out reach
        MoveReach(1,1800);
        //drive forward
        drivedeg(2250,2250,2250,2250,0.3,2000);
        //close mouth
        Servo_pos = 0.8;
        servo.setPosition(Servo_pos);
        //rasie boom
        MoveBoom(150,90,700);
        //back up
        //drivedeg(-300,-300,-300,-300,0.3,1000);
        //turn 90 deg
        SpinGyro(-85,0.3,3000,RightTurn);
        // check dist from wall using the right distance sensor (when on red alliance), strafe towards wall to 24" so we clear the bridge
        SFdist(24,0.4,distR,RightTurn);
        //drive forward to cross the under the bridge
        drivedeg(3100,3100,3100,3100,0.5,2500);
        //turn to put block out of the way
        SpinGyro(-45,0.3,3000,LeftTurn);
        //open claw
        Servo_pos = 0;
        servo.setPosition(Servo_pos);
        //spin back
        SpinGyro(-85,0.3,3000,RightTurn);
        //lower boom back down
        MoveBoom(-150,90,500);
        //backup
        drivedeg(-2950,-2950,-2950,-2950, 0.5, 2100);
        //spin to orient towards blocks
        SpinGyro(-5, 0.3, 3000, LeftTurn);
        //strafe to line up with 2nd block in - 29" from wall using the left distance sensor when on red alliance.
        SFdist(31,0.5,distL, LeftTurn);
        //moving forward to grab 2nd block
        drivedeg(350,350,350,350,0.3,1000);
        //move servo to secure block
        Servo_pos = 0.8;
        servo.setPosition(Servo_pos);
        //raise boom
        MoveBoom(150,90,700);
        //back up
       // drivedeg(-500,-500,-500,-500,0.3,1000);
        //spin right 90
        SpinGyro(-85,0.3,3000,RightTurn);
        //line up with bridge to park - 27" from wall
        SFdist(27,0.4,distR,LeftTurn);
        //drive forward
        drivedeg(3900,3900,3900,3900,0.9,2300);
        MoveBoom(150,50,1000);
        //open claw
        Servo_pos = 0;
        servo.setPosition(Servo_pos);
        //back up
        drivedeg(-1500,-1500,-1500,-1500, 0.8, 1200);
    }
    //__________________________________________________________________________________________________________________________

    public void test() {

    //   drivedeg(4000, 4000, 4000, 4000, 0.5, 10000);
        //SpinGyro(-90, .5, 5000, true);
        SFdist(12,0.5,true,true);
    }

    private void checkOrientation() {
// read the orientation of the robot
        angles = this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        this.imu.getPosition();
// and save the heading
        heading = angles.firstAngle;
    }
    public void runOpMode() {

        { //Prepare IMU to read gyro
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

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
            telemetry.addLine("Y for Fruit Loops - A for Waffles?");
            telemetry.update();
            while (QuestionAnswered == false) {
                if (gamepad1.y) {
                    //Fruit Loops!
                    telemetry.addLine("FRUIT LOOPS!!! Crunch Crunch!");
                    AutoL = true;
                    QuestionAnswered = true;
                }
                if (gamepad1.a){
                    //I guess we're having waffles for Breakfast
                    telemetry.addLine("WAFFLES! NUM NUM!");
                    AutoL = false;
                    QuestionAnswered = true;
                }
                telemetry.update();
            }
        AutoTransitioner.transitionOnStop(this, "FibbyTeleOp20");
        while (!isStarted()) {
            //Will run during INIT
            telemetry.addData("Encoders",  "Starting at %7d :%7d :%7d :%7d",
                    FR_drive.getCurrentPosition(),
                    FL_drive.getCurrentPosition(),
                    BR_drive.getCurrentPosition(),
                    BL_drive.getCurrentPosition());
            telemetry.update();

        }
        //Code beneath will start
        //Start autonomous period - 30 seconds to conquer the world!



        telemetry.update();


       if (AutoL == true) loops(Red_alliance);
       else if (AutoL == false) wafflesforbreakfast(Red_alliance);
     // test();
    }

        //turn off motors
        resetmotors();
    }

}