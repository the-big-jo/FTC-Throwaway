package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Basic: Linear OpMode", group="Linear OpMode")
public class BasicOpMode extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    // 4 motors
    private DcMotorEx backrightMotor = null;
    private DcMotorEx backleftMotor = null;
    private DcMotorEx frontrightMotor = null;
    private DcMotorEx frontleftMotor = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        // Uses the config of outreach7-10-26
        backrightMotor  = hardwareMap.get(DcMotorEx.class, "backright");
        backleftMotor  = hardwareMap.get(DcMotorEx.class, "backleft");
        frontrightMotor  = hardwareMap.get(DcMotorEx.class, "frontright");
        frontleftMotor  = hardwareMap.get(DcMotorEx.class, "frontleft");

        // Assumes 2 motors directly connected on the left and right, with each set to
        // rotate clockwise when fully powered in FORWARD mode
        backleftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontleftMotor.setDirection(DcMotor.Direction.REVERSE);
        backrightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontrightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double brPower;
            double blPower;
            double frPower;
            double flPower;

            double drive = -gamepad1.left_stick_y;
            double strafe  =  gamepad1.left_stick_x;
            double spin = - gamepad1.right_stick_y;
            double turn = gamepad1.right_stick_x;
            brPower = Range.clip(drive+strafe, -1.0, 1.0);
            blPower = Range.clip(drive-strafe, -1.0, 1.0);
            frPower = Range.clip(drive-strafe, -1.0, 1.0);
            flPower = Range.clip(drive+strafe, -1.0, 1.0);

            // Send calculated power to wheels
            backrightMotor.setPower(brPower);
            backleftMotor.setPower(blPower);
            frontrightMotor.setPower(frPower);
            frontleftMotor.setPower(flPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front Motors", "FL (%.2f), FR (%.2f)", flPower, frPower);
            telemetry.addData("Back Motors", "BL (%.2f), BR (%.2f)", blPower, brPower);
            telemetry.update();
        }
    }
}
