package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Locale;

@TeleOp(name="Fibbydrive19", group="Fibbydrive19")
@Disabled
public class fibbydrive19 extends OpMode {

    private DcMotor FL_drive = null;
    private DcMotor FR_drive = null;
    private DcMotor BL_drive = null;
    private DcMotor BR_drive = null;
    private DcMotor The_Boom = null;
    private DcMotor The_Boom2 = null;
    private CRServo servo1 = null;
    private Servo servo = null;
    double Servo_pos;
    DigitalChannel l_limit;
    DigitalChannel u_limit;

    // Run when the init button is hit
    // Run when the init button is hit
    public void reset_encoders ()
    {
        BR_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        The_Boom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FL_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        The_Boom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void init() {
     // Run when the init button is hit

        // INITIALIZATION

        //MediaPlayer mediaplayer = MediaPlayer.create(context, R.raw.gold);

        // define motor hardware map
        BR_drive = hardwareMap.dcMotor.get("FL_drive");
        BL_drive = hardwareMap.dcMotor.get("BL_drive");
        FR_drive = hardwareMap.dcMotor.get("FR_drive");
        FL_drive = hardwareMap.dcMotor.get("BR_drive");
        The_Boom = hardwareMap.dcMotor.get("The_Boom");
        The_Boom2 = hardwareMap.dcMotor.get("The_Boom2");
        servo1 = hardwareMap.crservo.get("servo1");
        servo = hardwareMap.servo.get("servo0");
        // set servo position to 1
        Servo_pos = 1;
        servo.setPosition(Servo_pos);

        // Lower Limit switch definition for Boom
        l_limit = hardwareMap.get(DigitalChannel.class, "l_limit");
        l_limit.setMode(DigitalChannel.Mode.INPUT);

        // Upper Limit switch definition for Boom
        u_limit = hardwareMap.get(DigitalChannel.class, "u_limit");
        u_limit.setMode(DigitalChannel.Mode.INPUT);

        // Status of the motor behavior
      //  boomMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);   // set to brake of float

        // define servo hardware map
      //  servo0 = hardwareMap.crservo.get("servo0");
      //  servo1 = hardwareMap.servo.get("servo1");

        // define sensors hardware map:

        // Lower Limit switch definition for Boom
      //  l_limit = hardwareMap.get(DigitalChannel.class, "l_limit");
      //  l_limit.setMode(DigitalChannel.Mode.INPUT);

        // Upper Limit switch definition for Boom
      //  u_limit = hardwareMap.get(DigitalChannel.class, "u_limit");
      //  u_limit.setMode(DigitalChannel.Mode.INPUT);

        // turn off all motors and reset servo positions to zero
        BR_drive.setPower(0);
        BL_drive.setPower(0);
        FR_drive.setPower(0);
        FL_drive.setPower(0);
        The_Boom.setPower(0);
        The_Boom2.setPower(0);
      //  boomMotor.setPower(0);
      //  servo0.setPower(0);

        // set servo position to 1
      //  servo1_pos = 1;
     //   servo1.setPosition(servo1_pos);

        // output telemetry data to screen
        telemetry.addData("Mode", "waiting");
        telemetry.update();

        // Reset Large Motor Encoders
        telemetry.addData("Status", "Resetting Encoders");    // outputting telemetry data
        telemetry.update();
        BR_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
     //   boomMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FL_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      //  boomMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                FR_drive.getCurrentPosition(),
                FL_drive.getCurrentPosition(),
                BR_drive.getCurrentPosition(),
                BL_drive.getCurrentPosition());
        telemetry.update();



        // ...
    }

    // run when the play button is hit
    public void start()
    {
        //mediaplayer.start();

        // Initialization of the Gyro
        // We added gyro heading telemetry outputs to aid us with writing pseudo code and
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU
       //imu = hardwareMap.get(BNO055IMU.class, "imu");
       //imu.initialize(parameters);

        // Set up our telemetry dashboard
        composeTelemetry();

        // Wait until we're told to go
        //waitForStart();

        // Start the logging of measured gyro acceleration
       //imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        // Loop and update the dashboard
        //while (opModeIsActive()) {
        //   telemetry.update();
        //}

        // test telemetry data
        //telemetry.addData("Whendoesthisrun", "Now");
        //composeTelemetry();
        //telemetry.update();


    }

    // main robot code
    public void loop()
    {
        int boom_power = 0;
        double left;
        double right;
        double Boom;
        // define variables to hold left joystick positions
        // these variables are continually updated as the loop continues
        if (gamepad1.right_stick_y != 0)
            right = gamepad1.right_stick_y * 0.5;
        else
            right = 0;

        if (gamepad1.left_stick_y != 0)
            left = gamepad1.left_stick_y * 0.5;
        else
            left = 0;

        // turbo button
        if (gamepad1.right_bumper || gamepad2.right_bumper) {
            left = left * 2;
            right = right * 2;
        }
        // slow button
        else if (gamepad1.left_bumper || gamepad2.left_bumper) {
            left = left * 0.5;
            right = right * 0.5;
        }

        if (gamepad2.left_stick_y != 0) {//if left stick is being moved
            Boom = gamepad2.left_stick_y;      // making the boom value the same as the gamepads left stick
            if (l_limit.getState() == true && Boom <= 0) {// if the l_limit had not been tripped and boom power is less than 0
                The_Boom.setPower(Boom);
                The_Boom2.setPower(Boom);
            }
            else if (u_limit.getState() == true && Boom >= 0){ //if the u_limit had not been tripped and boom power is more than 0
                The_Boom.setPower(Boom);
                The_Boom2.setPower(Boom);
            }
            else {
                The_Boom.setPower(0);
                The_Boom2.setPower(0);
            }
        }

        //reset encoders for pseudocode
       // if (gamepad1.b || gamepad2.b) {
        //    reset_encoders();
        //}
        // move servo based on boolean inputs from the 'A' and 'Y' buttons

        if ((gamepad1.y || gamepad2.y) && Servo_pos < 1) Servo_pos += 0.01;        // Y button
        else if ((gamepad1.a || gamepad2.a) && Servo_pos > 0) Servo_pos -= 0.01;   // A button
//Continuous rotation servo
        if (gamepad1.x || gamepad2.x)  servo1.setPower(1);        // X button - forward
        else if (gamepad1.b || gamepad2.b) servo1.setPower(0);   // B button - reverse
        else servo1.setPower(0.5); //- stop

        // update the servos position based on the inputs from the controller
        //servo0.setPower(contPower);
        servo.setPosition(Servo_pos);
        // code for changing motor rotation direction.
        // Motors on the right side of the robot need to have their rotation direction switched,
        // so this code changes the values of the set variables to negative
        BR_drive.setPower(-left);
        BL_drive.setPower(-left);
        FR_drive.setPower(-right);
        FL_drive.setPower(right);

        // define a variable to hold right joystick position


        // change servo power based on data from the left dpad
        // does not work, but we don't want to remove this before competition
        //  if (gamepad1.dpad_left) // boolean value
        //      contPower = .20;
        //  else if (gamepad1.dpad_right)
        //    contPower = -.20;
        // else
        //  contPower = 0.0;

        // move servo based on boolean inputs from the 'A' and 'Y' buttons

     //   if ((gamepad1.y || gamepad2.y) && servo1_pos < 1) servo1_pos += 0.01;        // Y button
       // else if ((gamepad1.a || gamepad2.a) && servo1_pos > 0) servo1_pos -= 0.01;   // A button

        // update the servos position based on the inputs from the controller
        //servo0.setPower(contPower);
       // servo1.setPosition(servo1_pos);



        // code for moving the boom
        // boom will only move if the limit switches are not triggered
        //double boom = gamepad2.right_stick_y;

        if (gamepad1.dpad_up || gamepad2.dpad_up)
            boom_power = -100;
        else if (gamepad1.dpad_down || gamepad2.dpad_down)
            boom_power = 100;
        else
            boom_power = 0;



        // output telemetry for the encoder data
        //telemetry.addData("Limit swich", l_limit.getState());
        telemetry.addData("Encoders RF RB LF LB",  "%7d :%7d :%7d :%7d",
                FR_drive.getCurrentPosition(),
                FL_drive.getCurrentPosition(),
                BR_drive.getCurrentPosition(),
                BL_drive.getCurrentPosition());
        telemetry.update();

        //display nacci's thrown position
       // telemetry.addData("Servo1", servo1.getPosition());
    }

    // telemetry stuff
    void composeTelemetry()
    {
        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable()
        {
            @Override
            public void run()
            {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
              //  angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                //gravity  = imu.getGravity();
            }
        });

        // output the heading data to the display
      //  telemetry.addData("heading", new Func<String>()
        {
          //  @Override
          //  public String value()

             //   return formatAngle(angles.angleUnit, angles.firstAngle);

        };
    }

    // Formatting data for the IMU/gyro

    String formatAngle(AngleUnit angleUnit, double angle)
    {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees)
    {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    // run when the stop button is pushed on the driver station
    public void stop()
    {
        // turn off a
        // l the motors
        BR_drive.setPower(0);
        BL_drive.setPower(0);
        FR_drive.setPower(0);
        FL_drive.setPower(0);
    }
}
