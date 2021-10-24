package org.firstinspires.ftc.teamcode;
import android.graphics.Color;
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

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;

import java.util.ArrayList;
@Autonomous(name="FibbyAuto22", group="Autonomous22")
public class FibbyAuto22 extends LinearOpMode {
    BNO055IMU imu;
    private DcMotor RF_drive = null;
    private DcMotor LF_drive = null;
    private DcMotor RB_drive = null;
    private DcMotor LB_drive = null;

    double LeftPower;
    double RightPower;
    double FrontPower;
    double BackPower;
    double heading = 0;
    double CorrectionFactor = 0.03;

    boolean Red_alliance = false;
    boolean FirstGyroRun = true;
    double rampmotorpower;
    boolean QuestionAnswered = false;

    double dst_heading;
    public ArrayList allTrackables;
    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

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
    //(ArrayList<VuforiaTrackable> allTrackables, TOOK THIS OUT WILL WANT LATER
    public void FIBBYSCREAM (boolean Red_Alliance) {

        boolean RightTurn = true;
        boolean LeftTurn = false;
        if (!Red_alliance) {
            RightTurn = false;
            LeftTurn = true;
        }
// ADD AUTO HERE
    }

    private void checkOrientation() {
// read the orientation of the robot
        angles = this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        this.imu.getPosition();
// and save the heading
        heading = angles.firstAngle;
    }
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

        //Mapping motor to variable names
        RF_drive = hardwareMap.dcMotor.get("RF_drive");
        LF_drive = hardwareMap.dcMotor.get("LF_drive");
        RB_drive = hardwareMap.dcMotor.get("RB_drive");
        LB_drive = hardwareMap.dcMotor.get("LB_drive");

        RF_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LF_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        AutoTransitioner.transitionOnStop(this, "FibbyTeleOp22");

        while (!isStarted()) {
            //FIBBYSCREAM(false);
        drivedeg(1000,100,10000,true,0,true);

        }
        //turn off motors
        resetmotors();
    }
}