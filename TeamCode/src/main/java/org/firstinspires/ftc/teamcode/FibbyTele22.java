package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Axis;

import java.util.Set;

@TeleOp(name="FibbyTeleOp22", group="FibbyTeleOp22")
public class FibbyTele22 extends OpMode {
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

    DigitalChannel xAxisStop;
    DigitalChannel yAxisStop;
   // private DcMotor DuckSpinner = null;
    //private Servo base;

    double RF_power = 0;
    double LF_power = 0;
    double RB_power = 0;
    double LB_power = 0;
    double xAxis_power = 0;
    double yAxis_power = 0;
    double Intake_power = 0;
    double DuckSpinner_power = 0;
    double timeleft;
    double drive = 0;
    double turn = 0;
    private ElapsedTime runtime = new ElapsedTime();
    boolean IntakeOn=false;
    boolean xAxisEncoder = false;
    boolean yAxisEncoder = false;
    boolean xAxisMoveComplete = true;
    boolean yAxisMoveComplete = true;
    boolean BoomMoveComplete = true;
    boolean BoomParkMoveRequest = false;
    int xDestinationPosition = 0;

    public void reset_encoders () {

        RF_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        xAxis.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        yAxis.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       // DuckSpinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        RF_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LF_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        xAxis.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        yAxis.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       // DuckSpinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void init () {
        RF_drive = hardwareMap.dcMotor.get("RF_drive");
        LF_drive = hardwareMap.dcMotor.get("LF_drive");
        RB_drive = hardwareMap.dcMotor.get("RB_drive");
        LB_drive = hardwareMap.dcMotor.get("LB_drive");
        xAxis = hardwareMap.dcMotor.get("xAxis");
        yAxis = hardwareMap.dcMotor.get("yAxis");
        Intake = hardwareMap.dcMotor.get("Intake");


        xAxisStop = hardwareMap.get(DigitalChannel.class, "xAxisStop");
        xAxisStop.setMode(DigitalChannel.Mode.INPUT);
        yAxisStop = hardwareMap.get(DigitalChannel.class, "yAxisStop");
        yAxisStop.setMode(DigitalChannel.Mode.INPUT);

        LeftDist = hardwareMap.get(DistanceSensor.class, "LeftDist");

        RightDist = hardwareMap.get(DistanceSensor.class, "RightDist");

        FrontDist = hardwareMap.get(DistanceSensor.class, "FrontDist");

        BackDist = hardwareMap.get(DistanceSensor.class, "BackDist");
      //  DuckSpinner = hardwareMap.dcMotor.get("DuckSpinner");
      //  base = hardwareMap.servo.get("base");

        LF_drive.setPower(0);
        RF_drive.setPower(0);
        LB_drive.setPower(0);
        RB_drive.setPower(0);
        xAxis.setPower(0);
        yAxis.setPower(0);
        Intake.setPower(0);
      //  DuckSpinner.setPower(0);

        RF_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        xAxis.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       // DuckSpinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LF_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        xAxis.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       // DuckSpinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        yAxis.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        yAxis.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        yAxis.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        yAxis.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        yAxis.setDirection(DcMotor.Direction.FORWARD);
        xAxis.setDirection(DcMotor.Direction.FORWARD);
        // send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                RF_drive.getCurrentPosition(),
                LF_drive.getCurrentPosition(),
                RB_drive.getCurrentPosition(),
                LB_drive.getCurrentPosition());
        telemetry.update();
        timeleft = 120;
        // Put boom home and reset encoders

        if (yAxisStop.getState()==true) //Is the arm all the way down on the limit switch?
        while (xAxisStop.getState()== false)  xAxis.setPower(0.2);//Move the arm right until the limit switch is triggered
        xAxis.setPower(0);
        xAxis.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        xAxis.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //
        xDestinationPosition=-38;
        xAxis.setTargetPosition(xDestinationPosition);
        xAxis.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        xAxis.setPower(0.2);
        while(xAxis.getCurrentPosition() >= xDestinationPosition+5)
        {}
        xAxis.setPower(0);
        xDestinationPosition=0;

        //yAxis.setPower(0.4);
        //while (yAxis.getCurrentPosition() < 500) {}
        //yAxis.setPower(0);
        xAxis.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        xAxis.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        yAxis.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
        public void start () {
            runtime.reset();
        }
            public void loop () {
            timeleft = 120 - runtime.seconds();
            if (gamepad1.right_stick_y != 0) {
                drive = -gamepad1.right_stick_y;
                turn = gamepad1.left_stick_x;


                //
                if (turn < 0) {
                    LF_power = drive + turn;
                    LB_power = drive + turn;
                    RF_power = drive;
                    RB_power = drive;
                }

                //
                else if (turn > 0) {
                    LF_power = drive;
                    LB_power = drive;
                    RF_power = drive - turn;
                    RB_power = drive - turn;
                } else {
                    LF_power = drive;
                    LB_power = drive;
                    RF_power = drive;
                    RB_power = drive;
                }
            }


            //
            else if (gamepad1.right_trigger != 0) {
                LF_power = gamepad1.right_trigger;
                LB_power = -gamepad1.right_trigger;
                RF_power = -gamepad1.right_trigger;
                RB_power = gamepad1.right_trigger;

            }
            //
            else if (gamepad1.left_trigger != 0) {
                LF_power = -gamepad1.left_trigger;
                LB_power = gamepad1.left_trigger;
                RF_power = gamepad1.left_trigger;
                RB_power = -gamepad1.left_trigger;
            }
            //i raised power from 0.2
            else if (gamepad1.right_bumper) {
                LF_power = 0.5; //+ gamepad1.left_stick_y;
                LB_power = 0.5;//+ gamepad1.left_stick_y;
                RF_power = -0.5;//+ gamepad1.left_stick_y;
                RB_power = -0.5;// + gamepad1.left_stick_y;
            }

            //moveing the motors indivigly to figer out what is what
            else if (gamepad1.left_bumper) {
                LF_power = -0.5;//+ gamepad1.left_stick_y;
                LB_power = -0.5;// + gamepad1.left_stick_y;
                RF_power = 0.5;// + gamepad1.left_stick_y;
                RB_power = 0.5;// + gamepad1.left_stick_y;
            }
            else
            // if ((gamepad1.right_stick_y == 0) && (gamepad1.left_bumper = false) && (gamepad1.right_bumper = false) && (gamepad1.right_trigger == 0) && (gamepad1.left_trigger == 0))
            {
                LF_power = 0;
                LB_power = 0;
                RF_power = 0;
                RB_power = 0;
                // boom = 0;
            }
            //spin the duck. sike we made that part of the intake hah!
              //  if (gamepad2.right_bumper) DuckSpinner.setPower(100);
               // else if (gamepad2.left_bumper) DuckSpinner.setPower(-100);

                //If the x Axis motor is set to run on encoder and the triggers are being touched, switch the motor to run without encoder
                if(((gamepad2.left_trigger !=0) || (gamepad2.right_trigger!=0)) && (xAxisEncoder==true))
                {
                    yAxisEncoder = false;
                    xAxisEncoder = false;
                    yAxis.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    xAxis.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    xAxisMoveComplete = true;
                    yAxisMoveComplete = true;
                    BoomMoveComplete = true;
                    BoomParkMoveRequest=false;
                }
                //power to turret
                if(gamepad2.left_trigger !=0) xAxis_power = -gamepad2.left_trigger;
                else if(gamepad2.right_trigger !=0 && xAxisStop.getState()== false) xAxis_power = gamepad2.right_trigger;
                else xAxis_power =0;

                //If the y Axis motor is set to run on encoder and the right stick is being pushed up or down, switch the motor to run without encoder
                if((gamepad2.right_stick_y !=0) && (yAxisEncoder==true))
                {
                    yAxisEncoder = false;
                    xAxisEncoder = false;
                    yAxis.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    xAxis.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    xAxisMoveComplete = true;
                    yAxisMoveComplete = true;
                    BoomMoveComplete = true;
                    BoomParkMoveRequest=false;
                }
                    //power to arm
                if ((gamepad2.right_stick_y < 0) || (gamepad2.right_stick_y > 0 && yAxisStop.getState()== false))
                   yAxis_power = gamepad2.right_stick_y;
                else
                    yAxis_power = 0;

                //boom presets
                if (gamepad2.a || gamepad2.b || gamepad2.x)//Set motors to run using encoder
                {
                    xAxisMoveComplete = true;
                    yAxisMoveComplete = true;
                    BoomMoveComplete = true;
                    BoomParkMoveRequest=false;
                    xAxisEncoder = true;
                    xAxis.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    yAxisEncoder = true;
                    yAxis.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        if (gamepad2.a)//boom go home
                        {
                            //Set boom height to clear obsticles
                            yAxis.setTargetPosition(1300);
                            yAxis.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            yAxis.setPower(0.5);
                            yAxisMoveComplete=false;
                            //Swing boom to home position - x Axis 0
                            xDestinationPosition=0; //0 is home
                            xAxisMoveComplete=false;
                            //Drop boom down to home position - y Axis 0
                            BoomMoveComplete=false;
                        }
                        else if(gamepad2.x)//boom left
                        {
                            //Set boom height to clear obsticles
                            yAxis.setTargetPosition(1300);
                            yAxis.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            yAxis.setPower(0.5);
                            yAxisMoveComplete=false;
                            //Swing boom to home position - x Axis 0
                            xDestinationPosition=-1100; //0 is home
                            xAxisMoveComplete=false;
                        }
                        else if (gamepad2.b)//boom right
                        {
                            //Set boom height to clear obsticles
                            yAxis.setTargetPosition(1300);
                            yAxis.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            yAxis.setPower(0.5);
                            yAxisMoveComplete=false;
                            //Swing boom to home position - x Axis 0
                            xDestinationPosition=1100; //0 is home
                            xAxisMoveComplete=false;

                }}

                //Watch for Boom preset move operations
                if (yAxisMoveComplete == false)
                {
                    if ((yAxis.getCurrentPosition() >= 1295) && (yAxis.getCurrentPosition() <= 1305))//Are we there yet??
                    {
                        yAxisMoveComplete=true;// Yes - yes we are there!
                        yAxis.setPower(0); //Close enough, kill the power
                        xAxis.setTargetPosition(xDestinationPosition);
                        xAxis.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        xAxis.setPower(0.3);//Now to move the turret into position
                }}
                if ((yAxisMoveComplete == true) && (xAxisMoveComplete == false))
                {
                    if(((xDestinationPosition < 0) && (xDestinationPosition+1) >= (xAxis.getCurrentPosition()))
                        || ((xDestinationPosition > 0 ) && (xDestinationPosition-1) <= (xAxis.getCurrentPosition()))
                        || ((xDestinationPosition == 0 ) && ((xDestinationPosition) <= (xAxis.getCurrentPosition()) && ((xDestinationPosition+2) >= (xAxis.getCurrentPosition())))))
                    {
                        //if we've achieved our position, turn the xAxis motor off and reset to allow control from the controller
                     xAxisMoveComplete = true;
                     xAxis.setPower(0);
                     xAxis.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                     xAxisEncoder = false;
                    }
                }

                if((xAxisMoveComplete) && (yAxisMoveComplete) && (!BoomMoveComplete)) //Should we drop the boom into it's home position? Only if the destination is 0
                {
                    yAxis.setTargetPosition(0);
                    yAxis.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    yAxis.setPower(0.3);
                    BoomParkMoveRequest=true;
                }
                if ((yAxis.getCurrentPosition() <= 10) && (BoomParkMoveRequest))
                {
                    BoomMoveComplete = true;
                    yAxisEncoder = false;
                    xAxisEncoder = false;
                    yAxis.setPower(0);
                    yAxis.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    xAxis.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    BoomParkMoveRequest=false;
                }
                //Intake and MAD DUCK SPINNER power
                if(gamepad2.dpad_down)
                    Intake_power = -0.5;
                else if(gamepad2.dpad_left)
                    Intake_power = -0.75;
                else if (gamepad2.dpad_up)
                    Intake_power = 1;
                else
                    Intake_power = 0;
                
                //set powers to motors
                RB_drive.setPower(-RB_power);
                RF_drive.setPower(-RF_power);
                // Motors aren't the same gear ratio, correction factor to left motors
                LB_drive.setPower(LB_power);
                LF_drive.setPower(LF_power);
                //Don't apply manual power to the motors if we're trying to move them to preset positions with encoder
if (xAxisEncoder==false)
                    xAxis.setPower(xAxis_power);
if (yAxisEncoder==false)
                    yAxis.setPower(-yAxis_power);

                Intake.setPower(Intake_power);
                telemetry.addData("Encoders X Y LF LB", "%7d :%7d :%7d :%7d",
                        xAxis.getCurrentPosition(),
                        yAxis.getCurrentPosition(),
                        RB_drive.getCurrentPosition(),
                        LB_drive.getCurrentPosition());


                telemetry.update();
        }
        
            public void stop() {
                // turn off all the motors
                RF_drive.setPower(0);
                LF_drive.setPower(0);
                RB_drive.setPower(0);
                LB_drive.setPower(0);
                xAxis.setPower(0);
                yAxis.setPower(0);
                Intake.setPower(0);
                yAxis.setPower(0);
                xAxis.setPower(0);
               // DuckSpinner.setPower(0);

            }

}

