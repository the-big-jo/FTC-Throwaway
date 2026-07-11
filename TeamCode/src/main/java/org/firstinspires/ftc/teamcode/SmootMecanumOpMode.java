package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class SmootMecanumOpMode extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    // 4 motors
    private DcMotorEx backrightMotor = null;
    private DcMotorEx backleftMotor = null;
    private DcMotorEx frontrightMotor = null;
    private DcMotorEx frontleftMotor = null;
    private GoBildaPinpointDriver odo = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        odo.setOffsets(0.0, 0.0, DistanceUnit.MM);
        odo.resetPosAndIMU();

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

            Pose2D pos = odo.getPosition();

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
            telemetry.addData("Odometry", "X: %.2f, Y: %.2f, H: %.2f", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.update();
        }
    }
}
