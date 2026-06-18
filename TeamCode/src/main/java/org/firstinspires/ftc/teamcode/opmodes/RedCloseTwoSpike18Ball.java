package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.utils.config.Constants;
import org.firstinspires.ftc.teamcode.utils.control.actions.ChangeStateAction;
import org.firstinspires.ftc.teamcode.utils.control.actions.FollowAction;
import org.firstinspires.ftc.teamcode.utils.control.actions.SleepAction;
import org.firstinspires.ftc.teamcode.utils.enums.Alliance;
import org.firstinspires.ftc.teamcode.utils.enums.RobotState;

public class RedCloseTwoSpike18Ball extends BaseAuto {
    @Override
    protected Alliance setColor() {
        return Alliance.RED;
    }

    @Override
    protected Pose setPose() {
        return Constants.RED_CLOSE_INIT;
    }

    @Override
    protected void setActionList() {
        Pose shootingPose = new Pose(89, 84.000);

        PathChain FirstShotSOTM = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(121.000, 124.000),
                                shootingPose
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(50), Math.toRadians(50))
                .build();

        PathChain path2ndSpike = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                shootingPose,
                                new Pose(82.000, 55.000),
                                new Pose(130.000, 58.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(50), Math.toRadians(0), .4)
                .build();
        PathChain  path2ndSpikeShot = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(130.000, 58.000),
                                new Pose(104.000, 66.000),
                                shootingPose
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        PathChain Gate = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                shootingPose,
                                new Pose(106.000, 46.000),
                                new Pose(135.000, 64.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-48), Math.toRadians(30), .6)
                .build();


        PathChain GateShot = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(132.000, 57.000),
                                shootingPose
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        PathChain FirstSpikeMark = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                shootingPose,
                                new Pose(125.000, 84.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        PathChain ShootFirstSpike = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(125.000, 82.000),
                                shootingPose
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        PathChain Finish = follower.pathBuilder()
                        .addPath(
                                new BezierLine(
                                        shootingPose,
                                        new Pose(105, 82)
                                )
                        )
                        .setTangentHeadingInterpolation()
                        .build();




        addAction(
                //PreLoad
                new ChangeStateAction(RobotState.SPEED_UP),
                new FollowAction(FirstShotSOTM),
                new ChangeStateAction(RobotState.FIRE),
                new SleepAction(500),
                new ChangeStateAction(RobotState.IDLE),

                //2nd Spike Mark
                new ChangeStateAction(RobotState.INTAKE),
                new FollowAction (path2ndSpike),
                new SleepAction(300),
                new ChangeStateAction(RobotState.SPEED_UP),
                new FollowAction(path2ndSpikeShot),
                new ChangeStateAction(RobotState.FIRE),
                new SleepAction(500),
                new ChangeStateAction(RobotState.IDLE),

                //1st Gate
                new ChangeStateAction(RobotState.INTAKE),
                new FollowAction(Gate, ()-> follower.getDistanceRemaining() < 8),
                new SleepAction(1300),
                new ChangeStateAction(RobotState.SPEED_UP),
                new FollowAction(GateShot),
                new SleepAction(450),
                new ChangeStateAction(RobotState.FIRE),
                new SleepAction(500),
                new ChangeStateAction(RobotState.IDLE),

                //2nd Gate
                new ChangeStateAction(RobotState.INTAKE),
                new FollowAction(Gate, ()-> follower.getDistanceRemaining() < 8),
                new SleepAction(1300),
                new ChangeStateAction(RobotState.SPEED_UP),
                new FollowAction(GateShot),
                new SleepAction(450),
                new ChangeStateAction(RobotState.FIRE),
                new SleepAction(500),
                new ChangeStateAction(RobotState.IDLE),

                //1st Spike
                new ChangeStateAction(RobotState.INTAKE),
                new FollowAction(FirstSpikeMark),
                new ChangeStateAction(RobotState.SPEED_UP),
                new FollowAction(ShootFirstSpike),
                new ChangeStateAction(RobotState.FIRE),
                new SleepAction(500),
                new ChangeStateAction(RobotState.IDLE),


                //3rd Gate
                new ChangeStateAction(RobotState.INTAKE),
                new FollowAction(Gate, ()-> follower.getDistanceRemaining() < 8),
                new SleepAction(1300),
                new ChangeStateAction(RobotState.SPEED_UP),
                new FollowAction(GateShot),
                new SleepAction(450),
                new ChangeStateAction(RobotState.FIRE),
                new SleepAction(500),
                new ChangeStateAction(RobotState.IDLE),

                //End Auto
                new FollowAction(Finish)
        );
    }
}
