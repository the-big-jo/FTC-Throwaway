package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static com.pedropathing.ivy.Scheduler.*;
import static com.pedropathing.ivy.pedro.PedroCommands.*;
import static com.pedropathing.ivy.groups.Groups.*;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name="ExampleAuto")
public class SimpleAuto extends LinearOpMode {

    private Follower follower;
    private PathChain MainChain;

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(new Pose(70.700, 22.500));


        waitForStart();

        schedule(autoRoutine());
        while (opModeIsActive()) {
            //Update the follower and execute the scheduler every loop
            follower.update();
            Scheduler.execute();
            // Feedback to Driver Hub for debugging
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.update();
        }
    }
    public void buildPaths(){
        MainChain = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(70.700, 22.500),
                                new Pose(37.102, 22.540),
                                new Pose(28.115, 74.437)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        new BezierCurve(
                                new Pose(28.115, 74.437),
                                new Pose(31.148, 127.094),
                                new Pose(106.876, 127.166),
                                new Pose(115.286, 83.093)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        new BezierCurve(
                                new Pose(115.286, 83.093),
                                new Pose(121.082, 59.612),
                                new Pose(71.077, 71.088)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();
    }
    public Command autoRoutine() {
        return sequential(
                follow(follower, MainChain, true)
        );
    }
}
