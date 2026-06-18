package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Switchback;
import org.firstinspires.ftc.teamcode.utils.ExternalTools;
import org.firstinspires.ftc.teamcode.utils.config.MatchDetails;

@Autonomous(preselectTeleOp = "TestTele")
public class AutoSelect extends OpMode {
    public Servo Light;
    private enum Options {
        AUTO("Auto"),
        CONFIG("Config");

        private String string;
        Options(String string) {
            this.string = string;
        }
    }

    private enum Auto {
        RED_CLOSE("Close Red"),
        RED_FAR("Far Red"),
        BLUE_CLOSE("Close Blue"),
        BLUE_FAR("Far BLue");

        private String string;
        Auto(String string) {
            this.string = string;
        }
    }

    private enum CloseConfig {
        ALL_SPIKE_12BALL("3 Spikes, 12 Balls, 1 Clear, 0 Gate"),
        TWO_SPIKE_18BALL("2 Spikes, 18 Balls, 3 Clears, 3 Gate"),
        TWO_SPIKE_12BALL("2 Spikes, 12 Balls, 2 Clears, 1 Gate"),
        THREE_SPIKE_18BALL("3 Spikes, 18 Balls, 2 Clears, 2 Gate");

        private String string;
        CloseConfig(String string) {
            this.string = string;
        }
    }

    private enum FarConfig {
        ONE_SPIKE_9BALL("1 Spike, 9 Balls"),
        ONE_SPIKE_12BALL("1 Spike, 12 Balls"),
        NO_SPIKE_9BALL("0 Spikes, 9 Balls"),
        NO_SPIKE_12BALL("0 Spike, 12 Balls");

        private String string;
        FarConfig(String string) {
            this.string = string;
        }
    }

    private Options     option;
    private Auto        auto;
    private CloseConfig closeConfig;
    private FarConfig   farConfig;

    private boolean settingsSet;

    protected Switchback switchback;

    private BaseAuto selectedAuto;

    @Override
    public void init() {
        option      = Options.AUTO;
        auto        = Auto.RED_CLOSE;
        closeConfig = CloseConfig.ALL_SPIKE_12BALL;
        farConfig   = FarConfig.ONE_SPIKE_9BALL;

        settingsSet = false;

        switchback = Switchback.getInstance();

        ExternalTools.initialize(telemetry);
    }

    @Override
    public void init_loop() {
        if (gamepad1.yWasPressed()) {
            selectedAuto = getSelectedAuto();
            selectedAuto.init(this);
            settingsSet = true;
        }

        if (!settingsSet) {
            if (gamepad1.dpadUpWasPressed() || gamepad1.dpadDownWasPressed()) {
                option = option == Options.CONFIG ? Options.AUTO : Options.CONFIG;
            }

            if (gamepad1.dpadRightWasPressed()) {
                if (option == Options.AUTO) {
                    auto = Auto.values()[(auto.ordinal()+1) % Auto.values().length];
                } else if (option == Options.CONFIG) {
                    if (auto == Auto.RED_CLOSE || auto == Auto.BLUE_CLOSE) {
                        closeConfig = CloseConfig.values()[(closeConfig.ordinal()+1) % CloseConfig.values().length];
                    } else {
                        farConfig = FarConfig.values()[(farConfig.ordinal()+1) % FarConfig.values().length];
                    }
                }
            } else if (gamepad1.dpadLeftWasPressed()) {
                if (option == Options.AUTO) {
                    auto = Auto.values()[(auto.ordinal()-1+Auto.values().length) % Auto.values().length];
                } else if (option == Options.CONFIG) {
                    if (auto == Auto.RED_CLOSE || auto == Auto.BLUE_CLOSE) {
                        closeConfig = CloseConfig.values()[(closeConfig.ordinal()-1+CloseConfig.values().length) % CloseConfig.values().length];
                    } else {
                        farConfig = FarConfig.values()[(farConfig.ordinal()-1+FarConfig.values().length) % FarConfig.values().length];
                    }
                }
            }

            ExternalTools.TELEMETRY.addData("Selected", option.string);
            ExternalTools.TELEMETRY.addData("Auto", auto.string);
            ExternalTools.TELEMETRY.addData("Config",
                    auto == Auto.RED_CLOSE || auto == Auto.BLUE_CLOSE
                    ? closeConfig.string : farConfig.string);
            ExternalTools.write();
        } else {
            switchback.read();
            if (gamepad1.aWasPressed()) {
//                switchback.getTurretSub().resetEncoder();
            }

            if (gamepad1.bWasPressed()) {
//                switchback.getTurretSub().setZeroToForwardAngle();
            }

            ExternalTools.TELEMETRY.addData("Angle", MatchDetails.zeroToForwardAngle);
            switchback.write();
        }
    }

    @Override
    public void start() {
        selectedAuto.start();
    }

    @Override
    public void loop() {
        selectedAuto.loop();

    }

    @Override
    public void stop() {
        if (settingsSet)
            selectedAuto.stop();
    }

    private BaseAuto getSelectedAuto() {
        switch (auto) {
            case RED_CLOSE -> {
                switch (closeConfig) {
                    case TWO_SPIKE_18BALL -> {
                        return new RedCloseTwoSpike18Ball();
                    }
                    case THREE_SPIKE_18BALL -> {
                        return new RedCloseThreeSpike18Ball();
                    }
                    default -> {
                        return new RedCloseThreeSpike12Ball();
                    }
                }
            }
            case BLUE_CLOSE -> {
                switch (closeConfig) {
                    case TWO_SPIKE_18BALL -> {
                        return new BlueCloseTwoSpike18Ball();
                    }
                    case THREE_SPIKE_18BALL -> {
                        return new BlueCloseThreeSpike18Ball();
                    }
                    default -> {
                        return new BlueCloseThreeSpike12Ball();
                    }
                }
            }
            case RED_FAR -> {
                switch (farConfig) {
                    case ONE_SPIKE_9BALL -> {
                        return new RedFarAuto();
                    }

                    case ONE_SPIKE_12BALL -> {
                        return new RedFar0pike();
                    }

                    default -> {
                        return new RedFarAuto();
                    }
                }
            }
            case BLUE_FAR -> {
                switch (farConfig) {
                    case ONE_SPIKE_9BALL -> {
                        return new BlueFarAuto();
                    }

                    case NO_SPIKE_9BALL -> {
                        return new BlueFar0pike();
                    }

                    default -> {
                        return new BlueFarAuto();
                    }
                }
            }
            default -> {
                ExternalTools.LOGGER.info("What the fuck.(Auto selection got borked)");
                return null;
            }
        }
    }
}
