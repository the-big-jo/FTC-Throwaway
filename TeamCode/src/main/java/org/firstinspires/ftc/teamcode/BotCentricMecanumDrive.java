package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

/*
    A simple op-mode that programs only movement. The op-mode
    uses a bot-centric drive, so the bot's orientation turns with the driving.

    The op-mode is 'smooth' because it is PID controlled, so
    the bot attempts to smooth out any drift/unintentional
    turning due to imperfect motors with PID. For example, if
    the bot starts slightly turning since one motor is weaker
    than the rest, that motor will be given more power to
    straighten out the bot.
 */

@TeleOp(name="BotCentricOpMode")
public class BotCentricMecanumDrive extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    // 4 motors
    private DcMotorEx backrightMotor = null;
    private DcMotorEx backleftMotor = null;
    private DcMotorEx frontrightMotor = null;
    private DcMotorEx frontleftMotor = null;
    private GoBildaPinpointDriver odo = null;
    private Limelight3A limelight = null;

    private double dt = 0.0f;

    @Override
    public void runOpMode() {
        // Uses the config of outreach7-10-26

        //telemetry setup
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //pinpoint odometry setup
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(0.0, 0.0, DistanceUnit.MM);
        odo.resetPosAndIMU();

        // motor setup
        backrightMotor  = hardwareMap.get(DcMotorEx.class, "backright");
        backleftMotor  = hardwareMap.get(DcMotorEx.class, "backleft");
        frontrightMotor  = hardwareMap.get(DcMotorEx.class, "frontright");
        frontleftMotor  = hardwareMap.get(DcMotorEx.class, "frontleft");
        backleftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontleftMotor.setDirection(DcMotor.Direction.REVERSE);
        backrightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontrightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        /* limelight setup
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();

         */

        // PID Values
        double P = 0;
        double I = 0;
        double D = 0;
        double lasterr = 0;

        double kP = 0; // PID coefficients set to 0 for now (needs bot testing)
        double kI = 0;
        double kD = 0;

        double desiredH = 0;

        waitForStart();
        runtime.reset();
        double time = runtime.time();

        // game loop
        while (opModeIsActive()) {
            // fetch data from odometry and limelight
            odo.update();
            Pose2D pos = odo.getPosition();

            // bot centric drive
            double drive = -gamepad1.left_stick_y;
            double strafe  =  gamepad1.left_stick_x;
            double spin = gamepad1.right_stick_x;
            double theta = odo.getHeading(UnnormalizedAngleUnit.RADIANS); // this needs to be UNNORMALIZED


            // normalizes power
            double dem = Math.max(Math.abs(drive) + Math.abs(strafe), 1);
            double brPower = Range.clip(drive+strafe, -1.0, 1.0) / dem;
            double blPower = Range.clip(drive-strafe, -1.0, 1.0) / dem;
            double frPower = Range.clip(drive-strafe, -1.0, 1.0) / dem;
            double flPower = Range.clip(drive+strafe, -1.0, 1.0) / dem;
            desiredH -= 3*spin*dt;

            // Adjustments
            double err = theta - desiredH;
            P = err;
            I += err*dt;
            D = lasterr-err;
            double adjust = P*kP + I*kI + D*kD;
            flPower += adjust;
            blPower += adjust;
            frPower -= adjust;
            brPower -= adjust;

            // Send calculated power to wheels
            backrightMotor.setPower(brPower);
            backleftMotor.setPower(blPower);
            frontrightMotor.setPower(frPower);
            frontleftMotor.setPower(flPower);


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Time: %.2f, dt: %.4f" + runtime.toString(), dt);
            telemetry.addData("Front Motors", "FL (%.2f), FR (%.2f)", flPower, frPower);
            telemetry.addData("Back Motors", "BL (%.2f), BR (%.2f)", blPower, brPower);
            telemetry.addData("Odometry", "X: %.2f, Y: %.2f, H: %.2f", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.update();

            // update dt
            dt = runtime.time() - time;
            time = runtime.time();
            lasterr = err;
        }
    }
}
