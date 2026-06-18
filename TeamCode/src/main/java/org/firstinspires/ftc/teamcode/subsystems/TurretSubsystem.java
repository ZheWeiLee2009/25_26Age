package org.firstinspires.ftc.teamcode.subsystems;


import android.annotation.SuppressLint;


import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.teamcode.utils.config.MatchDetails;
import org.firstinspires.ftc.teamcode.utils.control.ShootingWhileMoving;
import org.firstinspires.ftc.teamcode.utils.enums.Alliance;
import org.firstinspires.ftc.teamcode.utils.enums.RobotState;
import org.joml.Vector2d;


import java.util.function.IntSupplier;
import java.util.function.Supplier;


@Configurable
public class TurretSubsystem {
    private TurretStuff hardware;
    private final double TURRETFROMCENTERINCH = 57 / 25.4;
    //                                           millimeters / millimeter per inch
    public static boolean tracking = false, staticPos = false;

    public void init(TurretStuff hardware) {
        this.hardware = hardware;
    }


    public void read() {


    }


    @SuppressLint("DefaultLocale")
    public void update() {
        tracking = hardware.state.get() == RobotState.SPEED_UP
                || hardware.state.get() == RobotState.FIRE;
//        tracking=true;
        if (tracking) {
            double h = hardware.follower.getHeading();
            Vector2d robot = new Vector2d(hardware.follower.getPose().getX(), hardware.follower.getPose().getY());
            Vector2d target = new Vector2d(MatchDetails.target.x, MatchDetails.target.y);
            Vector2d turret = robot.sub(Math.cos(h) * TURRETFROMCENTERINCH, Math.sin(h) * TURRETFROMCENTERINCH);

            boolean isFar = robot.y < 45;


            if (isFar) {
                if(MatchDetails.ALLIANCECOLOR == Alliance.BLUE){
                    //if far blue, adjust target 13 inches to the right
                    target.add(18, 4);
                }else{
                    //if far red, adjust target 5 inches to the left
                    target.sub(18, 4);
                }

            } else {
                if(MatchDetails.ALLIANCECOLOR == Alliance.BLUE){
                    //if close blue, adjust target up 4 inches
//                    target.add(0, 4);
                }else{
                    //if close red, adjust target 4 inches to the right
//                    target.add(18, 0);
                }

            }


            double theta2 = Math.atan2(target.y - turret.y, target.x - turret.x);
            //double theta2 = Math.toRadians(-90);
            double theta3;
            // without swm
            theta3 = h - theta2;
            // with swm
//            theta3 = hardware.swm.getTurretAngleDeg();


            double angle = MatchDetails.zeroToForwardAngle + theta3 + (theta2-Math.PI > h ? 2*Math.PI : 0);


            double minAngle = Math.toRadians(-100), maxAngle = Math.toRadians(85), minServoPos = 0, maxServoPos = 0.59;
            double servoPos = ((angle-minAngle)/(maxAngle-minAngle)) * (maxServoPos-minServoPos) + minServoPos;


            hardware.turret.setPosition(Math.max(Math.min(servoPos, maxServoPos), minServoPos));


        } else {
            hardware.turret.setPosition(.32);
        }
    }


    public void write() {
    }


    public void setStaticPosition(boolean staticPos) {
        TurretSubsystem.staticPos = staticPos;
    }


    public record TurretStuff(
            Servo turret,
            IntSupplier turretPos,
            Follower follower,
            Supplier<RobotState> state,
            Supplier<Vector> robotVel,
            ShootingWhileMoving swm
    ) {}
}