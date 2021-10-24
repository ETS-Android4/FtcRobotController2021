package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="FibbyTeleOp22", group="FibbyTeleOp22")
public class FibbyTele22 extends OpMode {
    private DcMotor RF_drive = null;
    private DcMotor LF_drive = null;
    private DcMotor RB_drive = null;
    private DcMotor LB_drive = null;
    private DcMotor DuckSpinner = null;

    double RF_power = 0;
    double LF_power = 0;
    double RB_power = 0;
    double LB_power = 0;
    double DuckSpinner_POWER = 0;
    double timeleft;
    double drive = 0;
    double turn = 0;
    private ElapsedTime runtime = new ElapsedTime();


    public void reset_encoders () {

        RF_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DuckSpinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        RF_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LF_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DuckSpinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void init () {
        RF_drive = hardwareMap.dcMotor.get("RF_drive");
        LF_drive = hardwareMap.dcMotor.get("LF_drive");
        RB_drive = hardwareMap.dcMotor.get("RB_drive");
        LB_drive = hardwareMap.dcMotor.get("LB_drive");
        DuckSpinner = hardwareMap.dcMotor.get("DuckSpinner");

        LF_drive.setPower(0);
        RF_drive.setPower(0);
        LB_drive.setPower(0);
        RB_drive.setPower(0);
        DuckSpinner.setPower(0);

        RF_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DuckSpinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LF_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DuckSpinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                RF_drive.getCurrentPosition(),
                LF_drive.getCurrentPosition(),
                RB_drive.getCurrentPosition(),
                LB_drive.getCurrentPosition());
        telemetry.update();
        timeleft = 120;

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
                if (gamepad2.right_bumper) DuckSpinner.setPower(1);
                else if (gamepad2.left_bumper) DuckSpinner.setPower(-1);
                RB_drive.setPower(-RB_power);
                RF_drive.setPower(-RF_power);
                // Motors aren't the same gear ratio, correction factor to left motors
                LB_drive.setPower(LB_power);
                LF_drive.setPower(LF_power);
                DuckSpinner.setPower(DuckSpinner_POWER);
                telemetry.addData("Encoders RF RB LF LB", "%7d :%7d :%7d :%7d",
                        RF_drive.getCurrentPosition(),
                        LF_drive.getCurrentPosition(),
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
                DuckSpinner.setPower(0);

            }

}

