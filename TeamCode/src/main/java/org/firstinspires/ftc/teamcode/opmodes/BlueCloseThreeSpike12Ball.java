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

public class BlueCloseThreeSpike12Ball extends BaseAuto {
    @Override
    protected Alliance setColor() {
        return Alliance.BLUE;
    }

    @Override
    protected Pose setPose() {
        return Constants.BLUE_CLOSE_INIT;
    }

    @Override
    protected void setActionList() {
        Pose shootingPose = new Pose(48.000, 86.000);

        PathChain startToShooting = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(24.00, 127.700),

                                shootingPose
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(180))

                .build();

        PathChain shootingToPPG = follower.pathBuilder().addPath(
                        new BezierLine(
                                shootingPose,

                                new Pose(17, 84.500)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        PathChain PPGToLever = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(17, 84.500),
                                new Pose(34, 80.200),
                                new Pose(21.2, 76.500)
                        )
                )
                .setLinearHeadingInterpolation(180, Math.toRadians(180))
                .build();

        PathChain leverToShooting = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(21.2, 76.500),

                                shootingPose
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        PathChain shootingToPGP = follower.pathBuilder().addPath(
                        new BezierCurve(
                                shootingPose,
                                new Pose(50.300, 56.606),
                                new Pose(12.000, 60.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        PathChain PGPToShooting = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(12.000, 60.000),
                                new Pose(39.500, 66.500),
                                shootingPose
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        PathChain shootingToGPP = follower.pathBuilder().addPath(
                        new BezierCurve(
                                shootingPose,
                                new Pose(63, 37.200),
                                new Pose(12, 36.500)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        PathChain GPPToShooting = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(12, 36.500),

                                shootingPose.withY(100)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        addAction(
                new ChangeStateAction(RobotState.SPEED_UP),
                new FollowAction(startToShooting, () -> follower.getPose().getY() < 86.5),
                new SleepAction(800),
                new ChangeStateAction(RobotState.FIRE),
                new SleepAction(800),
                new ChangeStateAction(RobotState.IDLE),

                new ChangeStateAction(RobotState.INTAKE),
                new FollowAction(shootingToPPG, () -> follower.getPose().getX() < 18),
                new SleepAction(500),
                new ChangeStateAction(RobotState.IDLE),
                new SleepAction(250),

                new ChangeStateAction(RobotState.CLEAR),
                new FollowAction(PPGToLever),
                new SleepAction(100),

                new ChangeStateAction(RobotState.SPEED_UP),
                new FollowAction(leverToShooting, () -> follower.getPose().getX() > 47.5),
                new SleepAction(800),
                new ChangeStateAction(RobotState.FIRE),
                new SleepAction(1000),
                new ChangeStateAction(RobotState.IDLE),

                new ChangeStateAction(RobotState.INTAKE),
                new FollowAction(shootingToPGP, () -> follower.getPose().getX() < 13),
                new SleepAction(450),
                new ChangeStateAction(RobotState.IDLE),
                new SleepAction(250),

                new ChangeStateAction(RobotState.SPEED_UP),
                new FollowAction(PGPToShooting, () -> follower.getPose().getX() > 47.5),
                new SleepAction(800),
                new ChangeStateAction(RobotState.FIRE),
                new SleepAction(1000),
                new ChangeStateAction(RobotState.IDLE),

                new ChangeStateAction(RobotState.INTAKE),
                new FollowAction(shootingToGPP, () -> follower.getPose().getX() < 13),
                new SleepAction(500),
                new ChangeStateAction(RobotState.IDLE),
                new SleepAction(250),

                new ChangeStateAction(RobotState.SPEED_UP),
                new FollowAction(GPPToShooting, () -> follower.getPose().getX() > 47.5),
                new SleepAction(800),
                new ChangeStateAction(RobotState.FIRE),
                new SleepAction(1000),
                new ChangeStateAction(RobotState.IDLE),

                new ChangeStateAction(RobotState.IDLE)
        );
    }
}
