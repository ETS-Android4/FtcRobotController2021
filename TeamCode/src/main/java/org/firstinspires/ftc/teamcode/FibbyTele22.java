package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
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

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Axis;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.Set;
import java.util.concurrent.TimeUnit;

@TeleOp(name="FibbyTeleOp22", group="FibbyTeleOp22")
public class FibbyTele22 extends OpMode {
    //light controls
    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;

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

    DigitalChannel xAxisStop;
    DigitalChannel yAxisStop;
    DigitalChannel yAxisTopStop;

    double RF_power = 0;
    double LF_power = 0;
    double RB_power = 0;
    double LB_power = 0;
    double xAxis_power = 0;
    double yAxis_power = 0;
    double Intake_power = 0;
    double yAxis_position = 0;
    double xAxis_position = 0;
    //double DuckSpinner_power = 0;
    double timeleft;
    double drive = 0;
    double turn = 0;
    double Duck_Power;
    private ElapsedTime runtime = new ElapsedTime();
    boolean IntakeOn=false;
    boolean xAxisEncoder = false;
    boolean yAxisEncoder = false;
    boolean xAxisMoveComplete = true;
    boolean yAxisMoveComplete = true;
    boolean BoomMoveComplete = true;
    boolean BoomParkMoveRequest = false;
    boolean AutoMode = false;
    boolean IntakeFull = false;
    char AutoButtonPressed = 'o';
    int xDestinationPosition = 0;



    public void reset_encoders () {

        RF_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //xAxis.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       // yAxis.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        RF_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LF_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        xAxis.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        yAxis.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void init () {
        RF_drive = hardwareMap.dcMotor.get("RF_drive");
        LF_drive = hardwareMap.dcMotor.get("LF_drive");
        RB_drive = hardwareMap.dcMotor.get("RB_drive");
        LB_drive = hardwareMap.dcMotor.get("LB_drive");
        xAxis = hardwareMap.dcMotor.get("xAxis");
        yAxis = hardwareMap.dcMotor.get("yAxis");
        Intake = hardwareMap.dcMotor.get("Intake");

        // LED LIGHTS
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

        xAxisStop = hardwareMap.get(DigitalChannel.class, "xAxisStop");
        xAxisStop.setMode(DigitalChannel.Mode.INPUT);
        yAxisStop = hardwareMap.get(DigitalChannel.class, "yAxisStop");
        yAxisStop.setMode(DigitalChannel.Mode.INPUT);
        yAxisTopStop = hardwareMap.get(DigitalChannel.class, "yAxisTopStop");
        yAxisTopStop.setMode(DigitalChannel.Mode.INPUT);

        LeftDist = hardwareMap.get(DistanceSensor.class, "LeftDist");

        RightDist = hardwareMap.get(DistanceSensor.class, "RightDist");

        FrontDist = hardwareMap.get(DistanceSensor.class, "FrontDist");

        BackDist = hardwareMap.get(DistanceSensor.class, "BackDist");

        IntakeDist = hardwareMap.get(DistanceSensor.class, "IntakeDist");

        LF_drive.setPower(0);
        RF_drive.setPower(0);
        LB_drive.setPower(0);
        RB_drive.setPower(0);
        xAxis.setPower(0);





        yAxis.setPower(0);
        Intake.setPower(0);
      //  DuckSpinner.setPower(0);

        Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        yAxis.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        yAxis.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       // yAxis.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        yAxis.setDirection(DcMotor.Direction.FORWARD);
        xAxis.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        xAxis.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //xAxis.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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

        /*if (yAxisStop.getState()==true) //Is the arm all the way down on the limit switch?
        while (xAxisStop.getState()== false)  xAxis.setPower(0.2);//Move the arm right until the limit switch is triggered
        xAxis.setPower(0);
        xAxis.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        xDestinationPosition=-38;
        xAxis.setTargetPosition(xDestinationPosition);
        xAxis.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        xAxis.setPower(0.2);
        while(xAxis.getCurrentPosition() >= xDestinationPosition+5)
        {}
        xAxis.setPower(0);
        xDestinationPosition=0;
         */
        //xAxis.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        xAxis.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        yAxis.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        timeleft = 120;
        // ready to go just hit start
        pattern = RevBlinkinLedDriver.BlinkinPattern.SHOT_RED;
        blinkinLedDriver.setPattern(pattern);
    }
        public void start () {
            runtime.reset();
        }

            public void loop () {
            timeleft = 120 - runtime.seconds();
            if (gamepad1.right_stick_y != 0) {
                drive = -gamepad1.right_stick_y;
                turn = gamepad1.left_stick_x;

                if (turn < 0) {
                    LF_power = drive + turn;
                    LB_power = drive + turn;
                    RF_power = drive;
                    RB_power = drive;
                }


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
            }
            //spin the duck. sike we made that part of the intake hah!
              //  if (gamepad2.right_bumper) DuckSpinner.setPower(100);
               // else if (gamepad2.left_bumper) DuckSpinner.setPower(-100);
//==================================================================================================================================

                if (gamepad2.left_trigger != 0 || gamepad2.right_trigger !=0 || gamepad2.right_stick_y !=0) // is the driver trying to manually control
                {
                    if (AutoMode)
                    {
                        yAxis.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        xAxis.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        AutoMode = false;
                        AutoButtonPressed = 'p';
                    }

                    //Read game controller for motor powers
                    //power to turret
                    if(gamepad2.left_trigger !=0) xAxis_power = -gamepad2.left_trigger*0.35;
                    else if(gamepad2.right_trigger !=0 && xAxisStop.getState()== false)

                        xAxis_power = gamepad2.right_trigger*0.35;

                    else xAxis_power = 0;
                    //power to arm
                    if ((gamepad2.right_stick_y < 0 && yAxisTopStop.getState()== false) || (gamepad2.right_stick_y > 0 && yAxisStop.getState()== false))
                    {
                        yAxis_power = gamepad2.right_stick_y*0.4;
                    }
                        else
                        yAxis_power = 0;
                }

                else if (gamepad2.a || gamepad2.b || gamepad2.x || gamepad2.y|| gamepad2.right_bumper||gamepad2.left_bumper)
                {
                    if ((AutoButtonPressed == 'a' && gamepad2.a) || (AutoButtonPressed == 'b' && gamepad2.b) || (AutoButtonPressed == 'x' && gamepad2.x) || (AutoButtonPressed == 'y' && gamepad2.y)|| (AutoButtonPressed == 'r' && gamepad2.right_bumper)||(AutoButtonPressed == 'l' && gamepad2.left_bumper))
                    {} //We're using this if statement to prevent resetting the encoders and booleans multiple times when the bottom is held past the first loop through this code
                    //By doing this, we're preventing repeatedly resetting and causing false starts/stops. The interrupt functionality still works though.
                    else if (!AutoMode || AutoButtonPressed == 'a' || AutoButtonPressed == 'b' || AutoButtonPressed == 'x' || AutoButtonPressed == 'y'|| AutoButtonPressed == 'l'|| AutoButtonPressed == 'r') //We are looking for the AutoButtonPressed variable as a b or x to look for the buttom being pressed multiple times
                    {
                        yAxis.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        xAxis.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        yAxis.setPower(0);
                        xAxis.setPower(0);
                        AutoMode = true;
                        yAxisMoveComplete=true;
                        xAxisMoveComplete=true;
                        BoomMoveComplete=true;
                    }



                    if (gamepad2.a)//boom go home
                        /*
                        Step 1 - Move boom to position (1400) to avoid hitting parts on the robot
                        Step 2 - Swing boom to center
                        Step 3 - Lower boom to home position
                         */
                    {
                        AutoButtonPressed ='a';
                        if (yAxis.getCurrentPosition() <= 1405) {
                            yAxis.setTargetPosition(1400); //Step 1
                            yAxis.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            yAxis.setPower(0.6);
                        }
                        yAxisMoveComplete = false;
                        //Swing boom to home position - x Axis 0
                        xDestinationPosition=0; //0 is home - Step 2
                        xAxisMoveComplete=false;
                        //Drop boom down to home position - y Axis 0 //Step 3
                        BoomMoveComplete=false;
                    }
                    else if (gamepad2.x) //boom left
                    {
                        AutoButtonPressed ='x';
                        //Set boom height to clear obsticles
                        yAxis.setTargetPosition(1400);
                        yAxis.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        yAxis.setPower(0.7);
                        yAxisMoveComplete=false;
                        //Swing boom to home position - x Axis 0
                        xDestinationPosition=-2100; //0 is home
                        xAxisMoveComplete=false;
                    }
                    else if (gamepad2.b)//boom right
                    {
                        AutoButtonPressed ='b';
                        //Set boom height to clear obsticles
                        yAxis.setTargetPosition(1400);
                        yAxis.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        yAxis.setPower(0.7);
                        yAxisMoveComplete=false;
                        //Swing boom to home position - x Axis 0
                        xDestinationPosition=2100; //0 is home
                        xAxisMoveComplete=false;
                    }
                    else if (gamepad2.right_bumper)//boom right up
                    {
                        AutoButtonPressed ='r';
                        //Set boom height to clear obsticles
                        yAxis.setTargetPosition(2100);
                        yAxis.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        yAxis.setPower(0.7);
                        yAxisMoveComplete=false;
                        //Swing boom to home position - x Axis 0
                        xDestinationPosition=2100; //0 is home
                        xAxisMoveComplete=false;
                    }
                    else if (gamepad2.left_bumper)//boom left up
                    {
                        AutoButtonPressed ='l';
                        //Set boom height to clear obsticles
                        yAxis.setTargetPosition(2100);
                        yAxis.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        yAxis.setPower(0.7);
                        yAxisMoveComplete=false;
                        //Swing boom to home position - x Axis 0
                        xDestinationPosition=-2100; //0 is home
                        xAxisMoveComplete=false;
                    }
                    else if (gamepad2.y)// boom up for ducks
                    {
                        AutoButtonPressed = 'y';
                        //set boom height to clear robot and be the right height for the duck spinner
                        yAxis.setTargetPosition(1400);
                        yAxis.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        yAxis.setPower(0.7);
                        yAxisMoveComplete=false;
                        xDestinationPosition=0; //0 is home
                        xAxisMoveComplete=false;
                    }

                }
                else {
                    xAxis_power = 0;
                    yAxis_power = 0;
                }


                if (!AutoMode) //Only apply the motor powers if we're not in auto mode
                {
                    // dont forget to put back
                    xAxis.setPower(xAxis_power);
                    yAxis.setPower(-yAxis_power);
                }

                else if (AutoMode) //Execute on orders per Auto Mode
                {
                    switch (AutoButtonPressed) {
                        case 'a': //GO HOME
                            //(yAxis.getCurrentPosition() >= 1395) &&
                            if ((!yAxisMoveComplete) && (yAxis.getCurrentPosition() >= 1395))//Are we there yet??
                            {
                                yAxisMoveComplete=true;// Yes - yes we are there!
                                yAxis.setPower(0); //Close enough, kill the power
                                xAxis.setTargetPosition(xDestinationPosition); //Swing boom to center
                                xAxis.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                xAxis.setPower(0.6);//Now to move the turret into position{
                            }
                            else if (!xAxisMoveComplete && xAxis.getCurrentPosition() <= 2 && xAxis.getCurrentPosition() >= -2)
                            //Swing boom to home position - x Axis 0
                            {
                                //if we've achieved our position, turn the xAxis motor off and reset to allow control from the controller
                                xAxisMoveComplete = true;
                                xAxis.setPower(0);
                                yAxis.setTargetPosition(0);
                                yAxis.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                yAxis.setPower(0.3);
                            }
                            else if ((!BoomMoveComplete) && (yAxis.getCurrentPosition()<=4 || yAxisStop.getState()==true)) {
                                //Drop boom down to home position - y Axis 0 //Step 3
                                BoomMoveComplete=true;
                                yAxis.setPower(0);
                                AutoButtonPressed = 'p';
                            }
                            break;
                        case 'x':
                            if ((!yAxisMoveComplete) && (yAxis.getCurrentPosition() >= 1395) && (yAxis.getCurrentPosition() <= 1405))//Are we there yet??
                            {
                                yAxisMoveComplete=true;// Yes - yes we are there!
                                yAxis.setPower(0); //Close enough, kill the power
                                xAxis.setTargetPosition(xDestinationPosition); //Swing boom to center
                                xAxis.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                xAxis.setPower(0.8);//Now to move the turret into position{
                            }
                            else if (!xAxisMoveComplete && xAxis.getCurrentPosition()<=-2095)
                            {
                                xAxis.setPower(0);
                                xAxisMoveComplete=true;
                                AutoButtonPressed='p';
                            }
                            break;
                        case 'b':
                            if ((!yAxisMoveComplete) && (yAxis.getCurrentPosition() >= 1395) && (yAxis.getCurrentPosition() <= 1405))//Are we there yet??
                            {
                                yAxisMoveComplete=true;// Yes - yes we are there!
                                yAxis.setPower(0); //Close enough, kill the power
                                xAxis.setTargetPosition(xDestinationPosition); //Swing boom to center
                                xAxis.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                xAxis.setPower(0.8);//Now to move the turret into position{
                            }
                            else if (!xAxisMoveComplete && xAxis.getCurrentPosition()>=2095)
                            {
                                xAxis.setPower(0);
                                xAxisMoveComplete=true;
                                AutoButtonPressed='p';
                            }
                            break;

                        case 'l':
                            if ((!yAxisMoveComplete) && (yAxis.getCurrentPosition() >= 2095) && (yAxis.getCurrentPosition() <= 2105) && (yAxisTopStop.getState()==false))//Are we there yet??
                            {
                                yAxisMoveComplete=true;// Yes - yes we are there!
                                yAxis.setPower(0); //Close enough, kill the power
                                xAxis.setTargetPosition(xDestinationPosition); //Swing boom to center
                                xAxis.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                xAxis.setPower(0.8);//Now to move the turret into position{
                            }
                            else if (!xAxisMoveComplete && xAxis.getCurrentPosition()<=-2095)
                            {
                                xAxis.setPower(0);
                                xAxisMoveComplete=true;
                                AutoButtonPressed='p';
                            }
                            break;
                        case 'r':
                            if ((!yAxisMoveComplete) && (yAxis.getCurrentPosition() >= 2095) && (yAxis.getCurrentPosition() <= 2105) && (yAxisTopStop.getState()==false))//Are we there yet??
                            {
                                yAxisMoveComplete=true;// Yes - yes we are there!
                                yAxis.setPower(0); //Close enough, kill the power
                                xAxis.setTargetPosition(xDestinationPosition); //Swing boom to center
                                xAxis.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                xAxis.setPower(0.8);//Now to move the turret into position{
                            }
                            else if (!xAxisMoveComplete && xAxis.getCurrentPosition()>=2095)
                            {
                                xAxis.setPower(0);
                                xAxisMoveComplete=true;
                                AutoButtonPressed='p';
                            }
                            break;
                        case 'y':
                            //
                            if ((!yAxisMoveComplete) && (yAxis.getCurrentPosition() >= 1395) && (yAxis.getCurrentPosition() <= 1405))//Are we there yet??
                           {
                               yAxisMoveComplete=true;// Yes - yes we are there!
                               yAxis.setPower(0); //Close enough, kill the power
                               xAxis.setTargetPosition(xDestinationPosition); //Swing boom to center
                               xAxis.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                               xAxis.setPower(0.5);//Now to move the turret into position{
                           }
                            else if (!xAxisMoveComplete && xAxis.getCurrentPosition() <= 2 && xAxis.getCurrentPosition() >= -2)
                            //Swing boom to home position - x Axis 0
                            {
                                //if we've achieved our position, turn the xAxis motor off and reset to allow control from the controller
                                xAxisMoveComplete = true;
                                xAxis.setPower(0);
                                AutoButtonPressed='p';
                            }
                            break;
                        case 'p':
                            AutoMode = true;
                        default: //shouldn't ever happen
                    }
                }
//==================================================================================================================================

                //Intake and MAD DUCK SPINNER power
                //reseting the power if the button isnt pressed
                if (Duck_Power > 0.1 && !gamepad2.dpad_right)
                    Duck_Power = 0.3;
                //duck spinner with ramp up
                if (gamepad2.dpad_right) {
                    if (Duck_Power < 1) {
                        Duck_Power = Duck_Power + 0.1

                        ;
                        Intake_power = Duck_Power;
                    }
                }
                     // shooting out blocks
                else if(gamepad2.dpad_down)
                    Intake_power = -0.5;
                // shooting ball out
                else if(gamepad2.dpad_left)
                    Intake_power = -0.75;
                //grabing blocks and balls
                else if (gamepad2.dpad_up)
                    Intake_power = 1;
                //nothing
                else
                    Intake_power = 0;
                
                //set powers to motors
                RB_drive.setPower(-RB_power);
                RF_drive.setPower(-RF_power);
                // Motors aren't the same gear ratio, correction factor to left motors
                LB_drive.setPower(LB_power);
                LF_drive.setPower(LF_power);




                Intake.setPower(Intake_power);
                telemetry.addData("Encoders X Y LF LB", "%7d :%7d :%7d :%7d",
                        xAxis.getCurrentPosition(),
                        yAxis.getCurrentPosition(),
                        RB_drive.getCurrentPosition(),
                        LB_drive.getCurrentPosition());
                telemetry.addLine() .addData("IntakeDist", "%.3f",
                IntakeDist.getDistance(DistanceUnit.INCH));
                telemetry.update();
//light logic
                //Do we have a block?
                if (IntakeDist.getDistance(DistanceUnit.INCH) <= 3)
                {
                    //YES WE DO WOWOWOWOWO ALERT THE WORLD
                    IntakeFull = true;
                }
                //no :,(
                else IntakeFull =false;

                // is it telliop time?
                if (timeleft >= 45) {
                    // YES BLUE BLUE BLUE
                    //do we have a block?
                    if (!IntakeFull)
                    {
                        //no but the colors are pretty
                        pattern = RevBlinkinLedDriver.BlinkinPattern.CP1_2_COLOR_WAVES;
                        blinkinLedDriver.setPattern(pattern);
                    }
                    if (IntakeFull)
                    {
                        //YES ALERT EVERYONE AAAAA
                        pattern = RevBlinkinLedDriver.BlinkinPattern.STROBE_BLUE;
                        blinkinLedDriver.setPattern(pattern);
                    }
                }
                //is it endgame?
                if (timeleft <= 30) {
                    //yes RED ALERT ITS DUCK TIME BOIS GOGOGOGOGOGO
                    //do we have a block?
                    if (!IntakeFull)
                    {
                        //nope that's ok I guess
                        pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
                        blinkinLedDriver.setPattern(pattern);
                    }
                    if (IntakeFull)
                    {
                        //YES HAHAHHA LETS GOOOOO
                        pattern = RevBlinkinLedDriver.BlinkinPattern.STROBE_RED;
                        blinkinLedDriver.setPattern(pattern);
                    }
                }
                // 15 second End Game warning lights
                // is it 15 sec to endgame?
                else if (timeleft <= 45) {
                    //YES YELLOW ALERT
                    //do we have a block?
                    if (!IntakeFull) {
                        // no dang man get it together
                        pattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
                        blinkinLedDriver.setPattern(pattern);
                    }
                    else if(IntakeFull) {
                        // YES BLIND THEM AND GO
                        pattern = RevBlinkinLedDriver.BlinkinPattern.CP2_STROBE;
                        blinkinLedDriver.setPattern(pattern);
                    }
                }


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

