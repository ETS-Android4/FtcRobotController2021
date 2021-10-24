package org.firstinspires.ftc.teamcode;
//package com.arcrobotics.ftclib.hardware.motors;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

@Disabled
@TeleOp(name="FibbyTeleOp21", group="FibbyTeleOp21")

public class FibbyTele21 extends OpMode
{
    BNO055IMU imu;
    private DcMotor LF_drive = null;
    private DcMotor RF_drive = null;
    private DcMotor LB_drive = null;
    private DcMotor RB_drive = null;
    private DcMotor Thrower = null;
    private DcMotor Intake = null;
    private DcMotor TS = null;
    private DcMotor Arm = null;
    private Servo Trigger = null;
    private Servo Grabber = null;
    private Servo Lift = null;
    private ElapsedTime runtime = new ElapsedTime();
    DigitalChannel arm_limit;
    DigitalChannel arm_limit2;
    // private CRServo Spin1 = null;
    private DistanceSensor distleft;
    private DistanceSensor distright;
    private DistanceSensor distfront;
    private DistanceSensor distback;

    double LF_power = 0;
    double RF_power = 0;
    double LB_power = 0;
    double RB_power = 0;
    double thrower_V = 0;
    double Intake_power = 0;
    double TS_Power = 0;
    double ArmP = 0;
    double drive = 0;
    double turn = 0;
    double Servo_pos;
    double timeleft;
    boolean T_on = false;
    boolean G_on = false;
    boolean side = false;
    boolean FtB = false;
// auto pieces
    double LeftPower;
    double RightPower;
    double FrontPower;
    double BackPower;
    double heading = 0;
    double CorrectionFactor = 0.03;
    double BlueLineColor = 0.00;
    double RedLineColor = 1000;
    boolean Red_alliance = false;
    boolean FirstGyroRun = true;
    double rampmotorpower;
    double dst_heading;
    Orientation angles;
    ColorSensor colorSensor;

   // double velocity = Thrower.getVelocity();

    public void reset_encoders ()
    {
        LF_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Thrower.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LF_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Thrower.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        //Thrower.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Thrower.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    //-------------------------------------------------------------------------------------------------
    public void drivedist(int distance, double motorpower, int drivetime, boolean FrontSensor, int Course, boolean RampUp) {
        if (FrontSensor) {
            // Check front distance sensor to be sure we're not already too close
            if (distfront.getDistance(DistanceUnit.INCH) > distance) {
                // Leave motors powered until we close the distance to less than we passed in
                while ((distfront.getDistance(DistanceUnit.INCH) > distance)) {
                    GyroDriveBase(RampUp, Course, motorpower, false, false);
                }
            } else {

                // Leave motors powered until we close the distance to less than we passed in
                while ((distfront.getDistance(DistanceUnit.INCH) < distance)) {
                    GyroDriveBase(RampUp, Course, -motorpower, false, false);
                }
            }


        }
        //else is for using back sensor because the boolean frontsensor is false.
        else {
            // Check back distance sensor to be sure we're not already too close
            if (distback.getDistance(DistanceUnit.INCH) > distance) {
                // Leave motors powered until we close the distance to less than we passed in
                while ((distback.getDistance(DistanceUnit.INCH) > distance)) {
                    GyroDriveBase(RampUp, Course, -motorpower, false, false);
                }
            } else {
                // Leave motors powered until we expand the distance to more than we passed in
                while ((distback.getDistance(DistanceUnit.INCH) < distance)) {
                    GyroDriveBase(RampUp, Course, motorpower, false, false);
                }
            }
        }
        //turn off motors
        resetmotors();
        //pause to give motors a rest NO GRINDING GEARS
        FirstGyroRun = true;
        long setTime = System.currentTimeMillis();
        //public void loop() {
        if(System.currentTimeMillis() - setTime > (drivetime));

       // sleep(drivetime);
    }

    //-------------------------------------------------------------------------------------------------
    public void SFColor(boolean Red, double motorpower, boolean Right, boolean StopOnLine, int Course, boolean RampUp) {
        if (Red) {
            //This will strafe to the red line
            while ((colorSensor.red() < RedLineColor)) {
                GyroDriveBase(RampUp, Course, motorpower, true, Right);
            }
        } else {
            //This will strafe to the blue line
            while ((colorSensor.blue() < BlueLineColor)) {
                GyroDriveBase(RampUp, Course, motorpower, true, Right);
            }
        }

//turn off motors
        if (StopOnLine)
            resetmotors();
        FirstGyroRun = true;
    }

    public void DriveColor(boolean Red, double motorpower, boolean StopOnLine, boolean RampUp, int Course) {
        if (Red) {
            //This will drive backwards to the Red Line
            while ((colorSensor.red() < RedLineColor)) {
                GyroDriveBase(RampUp, Course, motorpower, false, false);
            }
        }
        // If Blue
        else {
            //This will drive to the Red Line
            while ((colorSensor.blue() < BlueLineColor)) {
                GyroDriveBase(RampUp, Course, motorpower, false, false);
            }
        }

//turn off motors if the boolean StopOnLine is true
        if (StopOnLine)
            resetmotors();
        FirstGyroRun = true;
    }

    //-------------------------------------------------------------------------------------------------
    public void SFdist(int dist, double motorpower, boolean distR, boolean StopMotors, boolean Right, int Course, boolean RampUp) {
        //If we want to use the LEFT distance sensor, then it will go through each of the following while loops to determine which direction to move.
        if (distR == false) {
            //This will strafe left using the left distance sensor moving TOWARD the object on the left
            if (!Right)
                while ((distleft.getDistance(DistanceUnit.INCH) >= dist)) {
                    GyroDriveBase(RampUp, Course, motorpower, true, Right);
                }
                //This will strafe right using the left distance sensor moving AWAY from the object on the left
            else while ((distleft.getDistance(DistanceUnit.INCH) <= dist)) {
                GyroDriveBase(RampUp, Course, motorpower, true, Right);
            }

        }
//Use the right Distance Sensor
        if (distR == true) {

            if (Right)
                while ((distright.getDistance(DistanceUnit.INCH) >= dist)) {
                    // Strafe Right
                    GyroDriveBase(RampUp, Course, motorpower, true, Right);
                }
            else while ((distright.getDistance(DistanceUnit.INCH) <= dist)) {
                //Strafe Left
                GyroDriveBase(RampUp, Course, motorpower, true, Right);
            }
        }

//turn off motors
        if (StopMotors)
            resetmotors();
        FirstGyroRun = true;
    }
    //-------------------------------------------------------------------------------------------------
    public void SpinGyro(double dest_heading, double motorpower, long drivetime, boolean Right, boolean ResetMotors) {
        if (Red_alliance == false) dest_heading = dest_heading * -1;
        long starttime = System.currentTimeMillis();
        checkOrientation();
        //If Right boolean is true it will spin right
        if (Right) {
            // Turn on the motors
            LF_drive.setPower(-motorpower);
            LB_drive.setPower(-motorpower);
            RF_drive.setPower(-motorpower);
            RB_drive.setPower(-motorpower);
            //Wait for heading to be achieved - if heading can't be achieved, timeout and exit the loop
            while ((heading >= dest_heading) && (System.currentTimeMillis() <= starttime + drivetime)) {
                checkOrientation();
            }
        }
        //If false it will spin left
        else {
            // Turn on the motors
            LF_drive.setPower(motorpower);
            LB_drive.setPower(motorpower);
            RF_drive.setPower(motorpower);
            RB_drive.setPower(motorpower);
            while ((heading <= dest_heading) && (System.currentTimeMillis() <= starttime + drivetime)) {
                checkOrientation();
            }
        }

        //turn off motors if the boolean ResetMotors is true.
        if (ResetMotors)
            resetmotors();
        FirstGyroRun = true;
    }


    public void init ()
        {
            // run when the init button is hit

            // INITIALIZATION


            // define motor hardware map
            LF_drive = hardwareMap.dcMotor.get("LF_drive");
            RF_drive = hardwareMap.dcMotor.get("RF_drive");
            LB_drive = hardwareMap.dcMotor.get("LB_drive");
            RB_drive = hardwareMap.dcMotor.get("RB_drive");
            Thrower = hardwareMap.dcMotor.get("thrower");
            //Thrower = new Motor(hardwareMap, "thrower", GoBILDA.RPM_6000);
            Intake = hardwareMap.dcMotor.get("intake");
            TS = hardwareMap.dcMotor.get("ts");
            Trigger = hardwareMap.servo.get("trigger");
            Grabber = hardwareMap.servo.get("grabber");
            Lift = hardwareMap.servo.get("lift");
            Arm = hardwareMap.dcMotor.get("arm");

            arm_limit = hardwareMap.get(DigitalChannel.class, "arm_limit");
            arm_limit.setMode(DigitalChannel.Mode.INPUT);

            arm_limit2 = hardwareMap.get(DigitalChannel.class, "arm_limit2");
            arm_limit2.setMode(DigitalChannel.Mode.INPUT);

            distleft = hardwareMap.get(DistanceSensor.class, "distleft");

            distright = hardwareMap.get(DistanceSensor.class, "distright");

            distfront = hardwareMap.get(DistanceSensor.class, "distfront");

            distback = hardwareMap.get(DistanceSensor.class, "distback");


            LF_drive.setPower(0);
            RF_drive.setPower(0);
            LB_drive.setPower(0);
            RB_drive.setPower(0);
            TS.setPower(0);
            Thrower.setPower(0);
            Intake.setPower(0);
            Arm.setPower(0);

            LF_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RF_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            LB_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RB_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            TS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Thrower.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            LF_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            RF_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            LB_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            RB_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            TS.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Thrower.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

    //-------------------------------------------------------------------------------------------------
    private void checkOrientation() {
// read the orientation of the robot
        angles = this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        this.imu.getPosition();
// and save the heading
        heading = angles.firstAngle;
    }

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
            LF_drive.setPower(-LeftPower);
            LB_drive.setPower(-LeftPower);
            RF_drive.setPower(RightPower);
            RB_drive.setPower(RightPower);
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
                LF_drive.setPower(-FrontPower);
                LB_drive.setPower(BackPower);
                RF_drive.setPower(-FrontPower);
                RB_drive.setPower(BackPower);
            } else {
                LF_drive.setPower(FrontPower);
                LB_drive.setPower(-BackPower);
                RF_drive.setPower(FrontPower);
                RB_drive.setPower(-BackPower);
            }
        }

    }

    //-------------------------------------------------------------------------------------------------
    //This function is used to consistently turn off powers to all of the motors and reset the encoders after every function.
    public void resetmotors() {
        RB_drive.setPower(0);
        LB_drive.setPower(0);
        RF_drive.setPower(0);
        LF_drive.setPower(0);
        Thrower.setPower(0);
        Intake.setPower(0);
        Arm.setPower(0);


        LF_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LF_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    //-------------------------------------------------------------------------------------------------
      public  void Auto_aim() {
          // turn to 0
          //SpinGyro(0, 0.5, 1000, true, true);
          //if we are closer to the right wall
          // && FtB == true
          if (distright.getDistance(DistanceUnit.INCH) > 27)
              SFdist(25,0.5,true,true,true,0,true);
          //if we are closer to the left wall
          else
              SFdist(25,0.5,true,true,false,0,true);

          //closer to front wall
        if(distback.getDistance(DistanceUnit.INCH) >44 || distfront.getDistance(DistanceUnit.INCH) < 40 )
              drivedist(44,0.5,1000,false,0,true);
          // closer to back wall
          else
          drivedist(44,0.5,1000,false,0,true);
          SpinGyro(-5,0.3,1000,true,true);
            Thrower.setPower(0.75);
            Lift.setPosition(0.6);
            long setTime = System.currentTimeMillis();
            while(System.currentTimeMillis() - setTime < 500) {}
            //shoot first ring
            Trigger.setPosition(1);
            setTime = System.currentTimeMillis();
            while(System.currentTimeMillis() - setTime < 500) {}
            Trigger.setPosition(0.5);
            setTime = System.currentTimeMillis();
            while(System.currentTimeMillis() - setTime < 500) {}
            //shoot second ring
            Trigger.setPosition(1);
            setTime = System.currentTimeMillis();
            while(System.currentTimeMillis() - setTime < 500) {}
            Trigger.setPosition(0.5);
            setTime = System.currentTimeMillis();
            while(System.currentTimeMillis() - setTime < 500) {}
            //shoot third ring
            Trigger.setPosition(1);
            setTime = System.currentTimeMillis();
            while(System.currentTimeMillis() - setTime < 500) {}
            Trigger.setPosition(0.5);
            Thrower.setPower(0);

      }
    //-------------------------------------------------------------------------------------------------
    public void Cangle(){
        SpinGyro(-5,0.3,1000,false,true);
        Lift.setPosition(0.6);
}
    //-------------------------------------------------------------------------------------------------
    public void start() {

        // gyro inputs from hub
        BNO055IMU.Parameters gyro_parameters = new BNO055IMU.Parameters();
        gyro_parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        gyro_parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        gyro_parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        gyro_parameters.loggingEnabled = true;
        gyro_parameters.loggingTag = "IMU";
        gyro_parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(gyro_parameters);

        //Color sensor prep
        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};
        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");
        colorSensor.enableLed(true);
        //controller inputs
        {
            //Servo_pos = 0.5;
            Trigger.setPosition(0.5);
            Grabber.setPosition(0.95);
            Lift.setPosition(0.5);

            // runtime.reset();
        }
    }
        public void loop() {
            timeleft = 120 - runtime.seconds();

            telemetry.addData("Encoders LF, RF, RB, LB", "Starting at %7d :%7d :%7d :%7d",
                    LF_drive.getCurrentPosition(),
                    RF_drive.getCurrentPosition(),
                    RB_drive.getCurrentPosition(),
                    LB_drive.getCurrentPosition());
            telemetry.update();

            if (gamepad1.right_stick_y != 0) {
                drive = -gamepad1.right_stick_y;
                turn = gamepad1.left_stick_x;


                //need fix
                if (turn < 0) {
                    LF_power = drive + turn;
                    RF_power = drive;
                    LB_power = drive + turn;
                    RB_power = drive;
                }

                //
                else if (turn > 0) {
                    LF_power = drive;
                    RF_power = drive - turn;
                    LB_power = drive;
                    RB_power = drive - turn;
                } else {
                    LF_power = drive;
                    RF_power = drive;
                    LB_power = drive;
                    RB_power = drive;
                }
            }
            //STRAFE RIGHT
            else if (gamepad1.right_trigger != 0) {
                LF_power = gamepad1.right_trigger;
                RF_power = -gamepad1.right_trigger;
                LB_power = -gamepad1.right_trigger;
                RB_power = gamepad1.right_trigger;

            }
            //STRAFE LEFT
            else if (gamepad1.left_trigger != 0) {
                LF_power = -gamepad1.left_trigger;
                RF_power = gamepad1.left_trigger;
                LB_power = gamepad1.left_trigger;
                RB_power = -gamepad1.left_trigger;
            }
            //i LOWRD power from 0.5
            //TURN RIGHT
            else if (gamepad1.right_bumper) {
                LF_power = 0.5; //+ gamepad1.left_stick_y;
                RF_power = -0.5;//+ gamepad1.left_stick_y;
                LB_power = 0.5;//+ gamepad1.left_stick_y;
                RB_power = -0.5;// + gamepad1.left_stick_y;
            }


            //TURN LEFT
            else if (gamepad1.left_bumper) {
                LF_power = -0.5;//+ gamepad1.left_stick_y;
                RF_power = 0.5;// + gamepad1.left_stick_y;
                LB_power = -0.5;// + gamepad1.left_stick_y;
                RB_power = 0.5;// + gamepad1.left_stick_y;
            }
            //moveing the motors indivigly to figer out what is what
            else if (gamepad1.dpad_left) LF_power = 100;
            else if (gamepad1.dpad_right) RF_power = 100;
            else if (gamepad1.dpad_up) LB_power = 100;
            else if (gamepad1.dpad_down) RB_power = 100;


                // if ((gamepad1.right_stick_y == 0) && (gamepad1.left_bumper = false) && (gamepad1.right_bumper = false) && (gamepad1.right_trigger == 0) && (gamepad1.left_trigger == 0))

            else {
                LF_power = 0;
                RF_power = 0;
                LB_power = 0;
                RB_power = 0;
            }

            if (gamepad2.left_bumper) {
                Intake_power = 100;
            } else if (gamepad2.right_bumper) {
                Intake_power = 0;
            }

            // a toggle for the thrower
            if (gamepad2.left_trigger > 0 && T_on == false) {
                thrower_V = 100;
                //was 75
                T_on = true;
            } else if (gamepad2.x && T_on == true) {
                thrower_V = 0;
                T_on = false;
            }

            if (gamepad2.right_trigger > 0) {
                Trigger.setPosition(1);
            } else Trigger.setPosition(0.5);
            /*if (gamepad2.dpad_left) {
                servo.setPosition(0.5);
            }
            if (gamepad2.dpad_down) {
                servo.setPosition(0);
            }

*/
            if (gamepad2.right_stick_y > 0 && arm_limit2.getState() == true) {
                ArmP = gamepad2.right_stick_y * 0.5;
            } else if (gamepad2.right_stick_y < 0 && arm_limit.getState() == true) {
                ArmP = gamepad2.right_stick_y * 0.5;
            } else ArmP = 0;

            if (gamepad2.b) {
                G_on = false;
                Grabber.setPosition(0.92);


            } else if (gamepad2.a) {
                //G_on = true;
                Grabber.setPosition(0.94);
            }


            if (gamepad2.dpad_up) {
                //was 0.5
                Lift.setPosition(0.6);
            } else if (gamepad2.dpad_left) {
                Lift.setPosition(0.5556); //was .555
            } else if (gamepad2.dpad_down) {

                Lift.setPosition(0.5);
            } else {
            }
            //press x to start self park
            if (gamepad1.x) {
                Cangle();
            }
           /*if (gamepad2.dpad_left){
               Spin1.setPower(1);
           }
           else if (gamepad2.dpad_right){
               Spin1.setPower(-1);
           }
           else {
               Spin1.setPower(0);
           }
          */

            LF_drive.setPower(-LF_power);
            RF_drive.setPower(RF_power);
            LB_drive.setPower(-LB_power);
            RB_drive.setPower(RB_power);
            //TS.setPower(TS_Power);
            // Thrower.setVelocity(thrower_V);
            Thrower.setPower(thrower_V);
            Intake.setPower(Intake_power);
            Arm.setPower(ArmP);

        }
    public void stop()
    {
        // turn off all the motors
        LF_drive.setPower(0);
        RF_drive.setPower(0);
        LB_drive.setPower(0);
        RB_drive.setPower(0);
        TS.setPower(0);
        Thrower.setPower(0);
        Intake.setPower(0);
        Arm.setPower(0);

    }

}



