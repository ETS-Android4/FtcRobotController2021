package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@TeleOp(name="FibbyPartyMode19", group="FibbyPartyMode19")

public class FibbyPartyMode19 extends OpMode {

    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;
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
    private CRServo testS = null;
    double RR_power = 0;
    double drive = 0;
    double turn = 0;
    double boom = 0;
    double LF_power = 0;
    double LR_power = 0;
    double RF_power = 0;
    double reach = 0;
    double Servo_pos;
    double chopsticks = 0;
    double boom_arm = 0;
    DigitalChannel l_limit;
    DigitalChannel u_limit;
    DigitalChannel f_limit;
    DigitalChannel c_limit;
    DigitalChannel arm_u_limit;
    DigitalChannel arm_l_limit;
    boolean resetBoom = false;
    private ElapsedTime runtime = new ElapsedTime();
    double timeleft;

    public void reset_encoders() {
        BR_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        The_Boom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        The_Boom2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Reach.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Boom_Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FL_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        The_Boom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        The_Boom2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Reach.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Boom_Arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Boom_Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        The_Boom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        The_Boom2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Reach.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void init() {
        // run when the init button is hit

        // INITIALIZATION

        //MediaPlayer mediaplayer = MediaPlayer.create(context, R.raw.gold);

        // define motor hardware map
        BR_drive = hardwareMap.dcMotor.get("BR_drive");
        BL_drive = hardwareMap.dcMotor.get("BL_drive");
        FR_drive = hardwareMap.dcMotor.get("FR_drive");
        FL_drive = hardwareMap.dcMotor.get("FL_drive");
        The_Boom = hardwareMap.dcMotor.get("The_Boom");
        The_Boom2 = hardwareMap.dcMotor.get("The_Boom2");
        Reach = hardwareMap.dcMotor.get("Reach");
        Boom_Arm = hardwareMap.dcMotor.get("Boom_Arm");
        servo = hardwareMap.servo.get("servo0");
        left_chop = hardwareMap.servo.get("left_chop");
        right_chop = hardwareMap.servo.get("right_chop");
        testS = hardwareMap.crservo.get("test");
        // set servo position to 1

        testS.setPower(0);
        // LED LIGHTS
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        // lower Limit switch definition for Boom
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

        BR_drive.setPower(0);
        BL_drive.setPower(0);
        FR_drive.setPower(0);
        FL_drive.setPower(0);
        The_Boom.setPower(0);
        The_Boom2.setPower(0);
        Reach.setPower(0);

        telemetry.addData("Mode", "waiting");
        telemetry.update();

        // reset Large Motor Encoders
        telemetry.addData("Status", "Resetting Encoders");    // outputting telemetry data
        telemetry.update();
        BR_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Boom_Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FL_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Boom_Arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                FR_drive.getCurrentPosition(),
                FL_drive.getCurrentPosition(),
                BR_drive.getCurrentPosition(),
                BL_drive.getCurrentPosition());
        telemetry.update();
        timeleft = 120;
        pattern = RevBlinkinLedDriver.BlinkinPattern.SHOT_RED;
        blinkinLedDriver.setPattern(pattern);

    }

    public void start() {
        pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_WITH_GLITTER;
        blinkinLedDriver.setPattern(pattern);
        runtime.reset();
        //composeTelemetry();

        Servo_pos = 0;
        servo.setPosition(Servo_pos);
        right_chop.setPosition(0.5);
        left_chop.setPosition(0.5);
    }

    public void loop() {
        timeleft = 120 - runtime.seconds();
        if (gamepad1.right_stick_y != 0) {
            drive = -gamepad1.right_stick_y;
            turn = gamepad1.left_stick_x;


            //
            if (turn < 0) {
                LF_power = drive + turn;
                LR_power = drive + turn;
                RF_power = drive;
                RR_power = drive;
            }

            //
            else if (turn > 0) {
                LF_power = drive;
                LR_power = drive;
                RF_power = drive - turn;
                RR_power = drive - turn;
            } else {
                LF_power = drive;
                LR_power = drive;
                RF_power = drive;
                RR_power = drive;
            }
        }


        //
        else if (gamepad1.right_trigger != 0) {
            LF_power = gamepad1.right_trigger;
            LR_power = -gamepad1.right_trigger;
            RF_power = -gamepad1.right_trigger;
            RR_power = gamepad1.right_trigger;

        }
        //
        else if (gamepad1.left_trigger != 0) {
            LF_power = -gamepad1.left_trigger;
            LR_power = gamepad1.left_trigger;
            RF_power = gamepad1.left_trigger;
            RR_power = -gamepad1.left_trigger;
        }
        //i raised power from 0.2
        else if (gamepad1.right_bumper) {
            LF_power = 0.5; //+ gamepad1.left_stick_y;
            LR_power = 0.5;//+ gamepad1.left_stick_y;
            RF_power = -0.5;//+ gamepad1.left_stick_y;
            RR_power = -0.5;// + gamepad1.left_stick_y;
        }

        //moveing the motors indivigly to figer out what is what
        else if (gamepad1.left_bumper) {
            LF_power = -0.5;//+ gamepad1.left_stick_y;
            LR_power = -0.5;// + gamepad1.left_stick_y;
            RF_power = 0.5;// + gamepad1.left_stick_y;
            RR_power = 0.5;// + gamepad1.left_stick_y;
        }
        /*
        else if (gamepad1.dpad_left) LF_power = 100;
        else if (gamepad1.dpad_right) RF_power = 100;
        else if (gamepad1.dpad_up) LR_power = 100;
        else if (gamepad1.dpad_down) RR_power = 100;
        */

        else
        // if ((gamepad1.right_stick_y == 0) && (gamepad1.left_bumper = false) && (gamepad1.right_bumper = false) && (gamepad1.right_trigger == 0) && (gamepad1.left_trigger == 0))
        {
            LF_power = 0;
            LR_power = 0;
            RF_power = 0;
            RR_power = 0;
            // boom = 0;
        }
//testing with a continues servo
        if (gamepad2.dpad_up) testS.setPower(1);
        else if (gamepad2.dpad_down) testS.setPower(0);
        else testS.setPower(0.5);

        if (gamepad1.dpad_up) {
            right_chop.setPosition(0.5);
            left_chop.setPosition(0.5);
        } else if (gamepad1.dpad_down) {
            right_chop.setPosition(1);
            left_chop.setPosition(0);
        }

        //boom power is equal to right joystick y
        if (gamepad2.b) resetBoom = true;

        if (gamepad2.right_stick_y != 0) {
            boom = gamepad2.right_stick_y;
            resetBoom = false;
        } else if (resetBoom && l_limit.getState() == true) boom = 1;
        else {
            resetBoom = false;
            boom = 0;
        }
        //If the lower limit switch is triggered and the boom is moving down then the boom power is 0.
        if (l_limit.getState() == false && boom >= 0) {
            boom = 0;
        }
        //If the uper limit switch is triggered and the boom is moving up then the boom power is 0.
        else if (u_limit.getState() == false && boom <= 0) {
            boom = 0;
        }
        //raised power from 0.1
        else if (boom > 0) boom = boom * 1;

        //boom power is equal to left joystick y

        //If the farther limit switch is triggered and the reach is moving out then the reach power is 0.
        if ((gamepad2.left_trigger != 0) && (c_limit.getState() == false)) {
            reach = -gamepad2.left_trigger;
        } else if ((gamepad2.right_trigger != 0) && (f_limit.getState() == false)) {
            reach = gamepad2.right_trigger;
        } else {
            reach = 0;
        }
        //
        boom_arm = (gamepad2.left_stick_y);

        if (arm_u_limit.getState() == false && boom_arm <= 0)
            boom_arm = gamepad2.left_stick_y;

            //If the lower limit switch is triggered and the boom is moving Down then the boom power is 0.
        else if (arm_l_limit.getState() == false && boom_arm >= 0)
            boom_arm = gamepad2.left_stick_y;

        else boom_arm = 0;


        //If gamepad2 y button is pressed and the servo position is less than 1 then the servo is adding 0.01 rotation
        if ((gamepad2.a) && Servo_pos < 1) Servo_pos += 0.01;        // Y button
            //If gamepad 2 a button is pressed the servo position is greater than 0 then the servo is subtracting 0.01 rotation
        else if ((gamepad2.y) && Servo_pos > 0) Servo_pos -= 0.01;   // A button
        //setting pos of servos using the bumpers
        if (gamepad2.right_bumper) Servo_pos = 0;
        else if (gamepad2.left_bumper) Servo_pos = 0.5;
        // update the servos position based on the inputs from the controller
        servo.setPosition(Servo_pos);


        //putting the variables where they belong
        //Apply power to motors
        BR_drive.setPower(-RR_power);
        FR_drive.setPower(-RF_power);
        // Motors aren't the same gear ratio, correction factor to left motors
        BL_drive.setPower(LR_power);
        FL_drive.setPower(LF_power);
        The_Boom.setPower(-boom);
        The_Boom2.setPower(-boom);
        Reach.setPower(-reach);
        Boom_Arm.setPower(-boom_arm);

        telemetry.addData("Encoders RF RB LF LB", "%7d :%7d :%7d :%7d",
                FR_drive.getCurrentPosition(),
                BR_drive.getCurrentPosition(),
                FL_drive.getCurrentPosition(),
                BL_drive.getCurrentPosition());

        telemetry.update();
        // Change light pattern for End Game
        if (timeleft <= 15) {
            pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_PARTY_PALETTE;
            blinkinLedDriver.setPattern(pattern);
        } else if (timeleft <= 30){
            pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_LAVA_PALETTE;
            blinkinLedDriver.setPattern(pattern);
        }
        // 15 second End Game warning lights
        else if (timeleft <= 45) {

            pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_FOREST_PALETTE;
            blinkinLedDriver.setPattern(pattern);
        } else if (timeleft <= 60) {
            runtime.reset();
        }
    }

    public void stop() {
        // turn off all the motors
        BR_drive.setPower(0);
        BL_drive.setPower(0);
        FR_drive.setPower(0);
        FL_drive.setPower(0);
        The_Boom.setPower(0);
        The_Boom2.setPower(0);
        Reach.setPower(0);
        Boom_Arm.setPower(0);
    }

}
