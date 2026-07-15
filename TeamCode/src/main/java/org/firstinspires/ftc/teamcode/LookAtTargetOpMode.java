package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
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

import java.util.List;

/*
    An op-mode meant to test out a LimeLight.

    When the user presses RB on the controller,the bot
    will turn towards the nearest visible April Tag. The
    bot will stay locked onto this April Tag until RT is
    pressed again. The bot will be able to move, but its
    heading will be uncontrollable while locked on. When
    the bot is locked on, it will use a field-centric
    drive. When it is not locked, it will use a bot-
    centric drive.
 */
@TeleOp(name="LookAtOpMode")
public class LookAtTargetOpMode extends LinearOpMode {
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

        //limelight setup
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();

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

        int targetting = 0;
        boolean lastRBdown = false;
        boolean lastLBdown = false;

        // game loop
        while (opModeIsActive()) {
            // fetch data from odometry and limelight
            odo.update();
            Pose2D pos = odo.getPosition();
            LLResult result = limelight.getLatestResult();

            // toggle targeting
            if (gamepad1.right_bumper) {
                if (!lastRBdown) {
                    targetting = (targetting==0) ? 24 : 0;
                }
                lastRBdown = true;
            } else {lastRBdown = false;}
            if (gamepad1.left_bumper) {
                if (!lastLBdown) {
                    targetting = (targetting==0) ? 20 : 0;
                }
                lastLBdown = true;
            } else {lastLBdown = false;}

            // field centric drive
            double drive = -gamepad1.left_stick_y;
            double strafe  =  gamepad1.left_stick_x;
            double spin = gamepad1.right_stick_x;

            // rotated directions
            double theta = odo.getHeading(UnnormalizedAngleUnit.RADIANS); // this needs to be UNNORMALIZED
            double Rdrive  = drive*Math.cos(-theta) - strafe*Math.sin(-theta);
            double Rstrafe = drive*Math.cos(-theta) + strafe*Math.sin(-theta);

            // normalizes power
            double dem = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(spin), 1);
            double brPower = Range.clip(Rdrive+Rstrafe, -1.0, 1.0) / dem;
            double blPower = Range.clip(Rdrive-Rstrafe, -1.0, 1.0) / dem;
            double frPower = Range.clip(Rdrive-Rstrafe, -1.0, 1.0) / dem;
            double flPower = Range.clip(Rdrive+Rstrafe, -1.0, 1.0) / dem;

            boolean visible = false;
            double angle = 0;
            if (result != null && result.isValid()) {
                List<FiducialResult> fiducials = result.getFiducialResults();
                for (FiducialResult fiducial : fiducials) {
                    int id = fiducial.getFiducialId();
                    if (id == targetting) {
                        angle = result.getTx();
                        visible = true;
                    }
                }
            }

            desiredH = visible ? theta : desiredH+3*spin*dt;

            // Adjustments
            double err = visible ? angle : theta - desiredH;
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
            telemetry.addData("Limelight", "Pipeline: , Targeting: %d, Tx: %.2f, Ty: %.2f", result.getPipelineIndex(), targetting, result.getTx(), result.getTy());
            telemetry.update();

            // update dt
            dt = runtime.time() - time;
            time = runtime.time();
            lasterr = err;
        }
    }
}
