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

public class RedCloseThreeSpike12Ball extends BaseAuto {
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
        Pose shootingPose = new Pose(96.000, 86.000);

        PathChain startToShooting = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(120.000, 127.700),

                                shootingPose
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(37), Math.toRadians(0))

                .build();

        PathChain shootingToPPG =  follower.pathBuilder().addPath(
                        new BezierLine(
                                shootingPose,

                                new Pose(127.000, 84.500)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        PathChain PPGToLever = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(127.000, 84.500),
                                new Pose(110.800, 80.200),
                                new Pose(122.800, 76.500)
                        )
                )
                .setLinearHeadingInterpolation(0, Math.toRadians(0))
                .build();

        PathChain leverToShooting = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(122.800, 76.500),

                                shootingPose
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        PathChain shootingToPGP = follower.pathBuilder().addPath(
                        new BezierCurve(
                                shootingPose,
                                new Pose(93.700, 56.606),
                                new Pose(132.000, 60.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        PathChain PGPToShooting = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(132.000, 60.000),
                                new Pose(104.500, 66.500),
                                shootingPose
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        PathChain shootingToGPP = follower.pathBuilder().addPath(
                        new BezierCurve(
                                shootingPose,
                                new Pose(81.000, 37.200),
                                new Pose(132.000, 36.500)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        PathChain GPPToShooting = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(132.000, 36.500),

                                shootingPose.withY(100)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        addAction(
                new ChangeStateAction(RobotState.SPEED_UP),
                new FollowAction(startToShooting, () -> follower.getPose().getY() < 86.5),
                new SleepAction(800),
                new ChangeStateAction(RobotState.FIRE),
                new SleepAction(800),
                new ChangeStateAction(RobotState.IDLE),

                new ChangeStateAction(RobotState.INTAKE),
                new FollowAction(shootingToPPG, () -> follower.getPose().getX() > 126),
                new SleepAction(500),
                new ChangeStateAction(RobotState.IDLE),
                new SleepAction(250),

                new ChangeStateAction(RobotState.CLEAR),
                new FollowAction(PPGToLever),
                new SleepAction(100),

                new ChangeStateAction(RobotState.SPEED_UP),
                new FollowAction(leverToShooting, () -> follower.getPose().getX() < 96.5),
                new SleepAction(800),
                new ChangeStateAction(RobotState.FIRE),
                new SleepAction(1000),
                new ChangeStateAction(RobotState.IDLE),

                new ChangeStateAction(RobotState.INTAKE),
                new FollowAction(shootingToPGP, () -> follower.getPose().getX() > 131.0),
                new SleepAction(450),
                new ChangeStateAction(RobotState.IDLE),
                new SleepAction(250),

                new ChangeStateAction(RobotState.SPEED_UP),
                new FollowAction(PGPToShooting, () -> follower.getPose().getX() < 96.5),
                new SleepAction(800),
                new ChangeStateAction(RobotState.FIRE),
                new SleepAction(1000),
                new ChangeStateAction(RobotState.IDLE),

                new ChangeStateAction(RobotState.INTAKE),
                new FollowAction(shootingToGPP, () -> follower.getPose().getX() > 131.0),
                new SleepAction(500),
                new ChangeStateAction(RobotState.IDLE),
                new SleepAction(250),

                new ChangeStateAction(RobotState.SPEED_UP),
                new FollowAction(GPPToShooting, () -> follower.getPose().getX() < 96.5),
                new SleepAction(800),
                new ChangeStateAction(RobotState.FIRE),
                new SleepAction(1000),
                new ChangeStateAction(RobotState.IDLE),

                new ChangeStateAction(RobotState.IDLE)
        );
    }
}
