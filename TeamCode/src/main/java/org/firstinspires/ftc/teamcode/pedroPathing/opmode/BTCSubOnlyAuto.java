package org.firstinspires.ftc.teamcode.pedroPathing.opmode;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

import java.util.concurrent.TimeUnit;


@Config
@Autonomous (name = "SubOnly", group = "AAA", preselectTeleOp = "AutoArmop"
)
public class BTCSubOnlyAuto extends BTCLibrary {
    private Telemetry telemetryA;


    public static double DISTANCE = 40;



    private Path forwards;
    private Path backwards;
    private Path push1;
    private Path push1ish;
    private Path push1ish2;
    private Path push2;
    private Path push2ish;
    private Path push3;
    private Path push3ish;
    private Path score1;
    private Path score2;
    private Path score2ish;
    private Path score3;
    private Path score3ish;
    private Path score4;
    private Path score4ish;
    private Path score5;
    private Path score7;
    private Path score8;
    private Path comeback1;
    private Path comeback4;
    private Path comeback5;
    private Path comeback6;
    private Path comeback7;
    private Path comeback8;
    private Path comeback1ish;
    private Path score6;
    private Path score6ish;
    private Path score6ish2;
    private Path comeback2;
    private Path comeback2ish;
    private Path comeback2ish2;
    private Path comeback3;
    private Path comeback3ish;
    private Path comeback3ish2;
    private PathChain MainCode;
    private PIDController controller;
    private PIDController armcontroller;


    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime drivetime = new ElapsedTime();

    public ElapsedTime flasher = new ElapsedTime();



    double mode = 1;
    double basketmove =1;
    double slideratio = 2;
    double slideticks = 103.8 * slideratio / 4.75;
    double armticks = 8192 / 360;
    double toplimit = 18.6;

    double bottomlimit = .25;
    double slidebasket = 1600 ;
    double armbasket = 2000;
    double twistbasket = .5;
    double wristbasket = .6;
    double slidespecimen = .5;
    double armspecimen = 1380 ;
    double wristspecimen = .3;
    double twistspecimen = .5;
    double armspecimenpickup = 60;
    double wristspecimenpickup = .51;
    double ticks = .002866;
    double xpress = 1;
    public double start = 0;
    public IMU imu = null;
    public DcMotor front_left = null;
    public DcMotor rear_left = null;
    public DcMotor front_right = null;
    public DcMotor rear_right = null;
    public double apress = 1;
    double just = 0;
    public RevTouchSensor limitfront;
    public RevTouchSensor limitfront2;
    public double r1press = 1;
    public double armPose = 0;
    double slidesPose = 0;
    public double count = 1;
    public DigitalChannel limitslide;
    public double dropping = 1;
    public double special_pick = 1;
    public double first_score = 1;
    public double stick = 1;
    public double missed = 0;

    public double safety = 1;
    private Path driveback = null;
    private Path driveback2 = null;
    private Path plus1 = null;
    public double ymod = 0;


    public double tx = 0;
    public double mod = 0;


    public boolean locked = false;
    public double ty = 0;
    public double breaak = 1;
    public boolean look = false;
    public Servo light = null;
    public boolean slidelimit = true;
    public Path forwardish = null;
    public double aapress = 1;
    public double a = 0;
    public double flipsafer = 1;
    public double good = 1;
    public double far = 8.5 * slideticks * 2;
    public double right = -8.5;
    public double left = 8.5;
    public double middle = 15.125;
    public double shift = 0;
    public double armadjust = 0;
    public double twistadjust = 0;
    public boolean autotwist = true;
    public String pipeline = "";
    public boolean plus = false;
    public RevColorSensorV3 side = null;

    @Override
    public void init(){
        initialize();
        follower.setStartingPose(new Pose(0, 6));
        backwards = new Path(new BezierLine(new Point(DISTANCE,0, Point.CARTESIAN), new Point(0,0, Point.CARTESIAN)));
        backwards.setConstantHeadingInterpolation(0);
        score1 = new Path(new BezierLine(new Point(0, 6, Point.CARTESIAN), new Point(36, 18, Point.CARTESIAN)));
        score1.setConstantHeadingInterpolation(0);
        score2 = new Path(new BezierCurve(new Point(4, -18.5, Point.CARTESIAN), new Point(9.1, 0, Point.CARTESIAN), new Point(12.3, 10, Point.CARTESIAN), new Point(23.3, 12, Point.CARTESIAN), new Point(36, 14.2, Point.CARTESIAN)));
        score2.setConstantHeadingInterpolation(0);
        push1 = new Path(new BezierCurve(new Point(22, 9.3, Point.CARTESIAN),new Point(17.1, -2.48, Point.CARTESIAN), new Point(16.24, -15, Point.CARTESIAN), new Point(21.8, -20, Point.CARTESIAN), new Point(38, -20, Point.CARTESIAN),new Point(48.2, -20, Point.CARTESIAN),new Point(48.8, -30, Point.CARTESIAN),new Point(38.8, -31, Point.CARTESIAN)));
        push1ish = new Path(new BezierCurve(new Point(37, -31, Point.CARTESIAN), new Point(30.2, -30, Point.CARTESIAN), new Point(20.2, -30, Point.CARTESIAN), new Point(16.9, -30, Point.CARTESIAN)));
        push1.setConstantHeadingInterpolation(0);
        push1ish.setConstantHeadingInterpolation(0);
        push2 = new Path(new BezierCurve(new Point(14.9, -31, Point.CARTESIAN),new Point(27.2, -31, Point.CARTESIAN), new Point(39.5, -31, Point.CARTESIAN), new Point(44.7, -32, Point.CARTESIAN),new Point(47.5, -39.6, Point.CARTESIAN)));
        push2ish = new Path(new BezierCurve(new Point(47.5, -39.6, Point.CARTESIAN), new Point(44.7, -39.6, Point.CARTESIAN), new Point(30.5, -39.7, Point.CARTESIAN), new Point(16.8, -40, Point.CARTESIAN)));
        push2.setConstantHeadingInterpolation(0);
        push2ish.setConstantHeadingInterpolation(0);
        push3 = new Path(new BezierCurve(new Point(13.8, -40, Point.CARTESIAN), new Point(27.3, -39.9, Point.CARTESIAN), new Point(39, -40.7, Point.CARTESIAN), new Point(42, -41.3, Point.CARTESIAN),new Point(44.6, -47.8, Point.CARTESIAN)));
        push3ish = new Path(new BezierCurve(new Point(50.6, -46.8, Point.CARTESIAN), new Point(39.2, -46.8, Point.CARTESIAN), new Point(16.7, -46.8, Point.CARTESIAN), new Point(4, -46.8, Point.CARTESIAN)));
        push3.setConstantHeadingInterpolation(0);
        push3ish.setConstantHeadingInterpolation(0);
        score2 = new Path(new BezierCurve(new Point(5, -18.5, Point.CARTESIAN), new Point(9.1, 0, Point.CARTESIAN), new Point(12.3, 10, Point.CARTESIAN), new Point(23.3, 12, Point.CARTESIAN), new Point(36, 16.5, Point.CARTESIAN)));
        score2.setConstantHeadingInterpolation(0);
        comeback1 = new Path(new BezierCurve(new Point(28, 18, Point.CARTESIAN), new Point(23.5, 8, Point.CARTESIAN), new Point(18.9, -12.3, Point.CARTESIAN), new Point(11, -16.8, Point.CARTESIAN)));
        comeback1.setConstantHeadingInterpolation(0);
        comeback2 = new Path(new BezierCurve(new Point(27, 16.5, Point.CARTESIAN), new Point(14.9, -15.3, Point.CARTESIAN), new Point(10, -18.8, Point.CARTESIAN)));
        comeback2.setConstantHeadingInterpolation(0);
        comeback3 = new Path(new BezierCurve(new Point(28, 15, Point.CARTESIAN), new Point(23.5, 8, Point.CARTESIAN), new Point(18.9, -12.3, Point.CARTESIAN), new Point(11, -16.8, Point.CARTESIAN)));
        comeback3.setConstantHeadingInterpolation(0);
        comeback4 = new Path(new BezierCurve(new Point(28, 14.5, Point.CARTESIAN), new Point(23.5, 8, Point.CARTESIAN), new Point(18.9, -12.3, Point.CARTESIAN), new Point(11, -16.8, Point.CARTESIAN)));
        comeback4.setConstantHeadingInterpolation(0);
        comeback5 = new Path(new BezierCurve(new Point(28, 13, Point.CARTESIAN), new Point(23.5, 8, Point.CARTESIAN), new Point(18.9, -12.3, Point.CARTESIAN), new Point(11, -16.8, Point.CARTESIAN)));
        comeback5.setConstantHeadingInterpolation(0);
        comeback6 = new Path(new BezierCurve(new Point(28, 12, Point.CARTESIAN), new Point(23.5, 8, Point.CARTESIAN), new Point(18.9, -12.3, Point.CARTESIAN), new Point(11, -16.8, Point.CARTESIAN)));
        comeback6.setConstantHeadingInterpolation(0);
        comeback7 = new Path(new BezierCurve(new Point(28, 12, Point.CARTESIAN), new Point(23.5, 8, Point.CARTESIAN), new Point(18.9, -12.3, Point.CARTESIAN), new Point(11, -16.8, Point.CARTESIAN)));
        comeback7.setConstantHeadingInterpolation(0);
        comeback8 = new Path(new BezierCurve(new Point(28, 11, Point.CARTESIAN), new Point(23.5, 8, Point.CARTESIAN), new Point(18.9, -12.3, Point.CARTESIAN), new Point(11, -16.8, Point.CARTESIAN)));
        comeback8.setConstantHeadingInterpolation(0);
        comeback1ish = new Path(new BezierCurve(new Point(11, -18, Point.CARTESIAN), new Point(6, -18.5, Point.CARTESIAN), new Point(4, -18.5, Point.CARTESIAN)));
        comeback1ish.setConstantHeadingInterpolation(0);
        comeback2ish = new Path(new BezierLine(new Point(11, -17.5, Point.CARTESIAN), new Point(2, -30.5, Point.CARTESIAN)));
        comeback2ish.setConstantHeadingInterpolation(0);
        comeback2 = new Path(new BezierCurve(new Point(28, 15.4, Point.CARTESIAN), new Point(23.5, 8, Point.CARTESIAN), new Point(18.9, -12.3, Point.CARTESIAN), new Point(11, -16.8, Point.CARTESIAN)));
        comeback2.setConstantHeadingInterpolation(0);
        score3 = new Path(new BezierCurve(new Point(5, -18.5, Point.CARTESIAN), new Point(9.1, 0, Point.CARTESIAN), new Point(12.3, 10, Point.CARTESIAN), new Point(23.3, 12, Point.CARTESIAN), new Point(36, 15, Point.CARTESIAN)));
        score3.setConstantHeadingInterpolation(0);
        score4 = new Path(new BezierCurve(new Point(5, -18.5, Point.CARTESIAN), new Point(9.1, 0, Point.CARTESIAN), new Point(12.3, 10, Point.CARTESIAN), new Point(23.3, 12, Point.CARTESIAN), new Point(36, 14.5, Point.CARTESIAN)));
        score4.setConstantHeadingInterpolation(0);
        score5 = new Path(new BezierCurve(new Point(5, -18.5, Point.CARTESIAN), new Point(9.1, 0, Point.CARTESIAN), new Point(12.3, 10, Point.CARTESIAN), new Point(23.3, 12, Point.CARTESIAN), new Point(36, 13, Point.CARTESIAN)));
        score5.setConstantHeadingInterpolation(0);
        score6 = new Path(new BezierCurve(new Point(5, -18.5, Point.CARTESIAN), new Point(9.1, 0, Point.CARTESIAN), new Point(12.3, 10, Point.CARTESIAN), new Point(23.3, 12, Point.CARTESIAN), new Point(36, 12, Point.CARTESIAN)));
        score6.setConstantHeadingInterpolation(0);
        score7 = new Path(new BezierCurve(new Point(5, -18.5, Point.CARTESIAN), new Point(9.1, 0, Point.CARTESIAN), new Point(12.3, 10, Point.CARTESIAN), new Point(23.3, 12, Point.CARTESIAN), new Point(36, 12, Point.CARTESIAN)));
        score7.setConstantHeadingInterpolation(0);
        score8 = new Path(new BezierCurve(new Point(5, -18.5, Point.CARTESIAN), new Point(9.1, 0, Point.CARTESIAN), new Point(12.3, 10, Point.CARTESIAN), new Point(23.3, 12, Point.CARTESIAN), new Point(36, 12, Point.CARTESIAN)));
        score8.setConstantHeadingInterpolation(0);
        score3ish = new Path(new BezierLine(new Point(28, 11.6, Point.CARTESIAN), new Point(31, 15.4, Point.CARTESIAN)));
        score3ish.setConstantHeadingInterpolation(0);
        follower.followPath(score1);
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        gamepad2.setLedColor(255,0,255,999999999);
        gamepad1.setLedColor(255,162,0,999999999);
        armtarget = 0;
        slidestarget = 0;
        wristpose = .8;
        flippose = .21;
        twistpose = 0;
    }

    /**
     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
     * initializes the FTC Dashboard telemetry.
     */
    @Override
    public void init_loop() {
        arm();
    }


    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the FTC Dashboard.
     */
    @Override
    public void loop() {
        follower.telemetryDebug(telemetryA);
        if(a == 0){
            a = 1;
            armtarget = 732;
            slidestarget = 648;
            wristpose = .78;
            twistpose = 0;
            flippose = .653;
            intake_turret_pos = .3;
            intake_vertical_pos = ready_pose;
            gripspinny.setPower(-1);
        }
        arm();
        safeflip();
        getwall();
        takepicture();
        drop_at_human_player();
        slideservomove();
        intake_turret_move();
        griptwist();
        intake_vertical_move();
        if(forward == 1 || forward == 9 || forward == 9.5){
            if(follower.getPose().getX() > 20){
                if(count != 7 || count != 1) {
                    follower.setMaxPower(.4);
                }else{
                    follower.setMaxPower(.7);
                }
            }
            if(follower.getPose().getX() > 31){
                gripspinny.setPower(1);
                slidestarget = 0;
                wristpose = .5;
            }
        }else{
            follower.setMaxPower(1);
        }
        if(forward == 10 || forward == 20){
            if(follower.getPose().getX() < 29){
                armtarget = 732;
                wristpose = .43;
                slidestarget = 0;
                flippose = .025;
                if(follower.getPose().getX() < 25){
                    intake_turret_pos = turret_drop;
                    intake_vertical_pos = drop_pose;
                }

                twistpose = .56;
            }
        }
        follower.update();
        if(follower.atParametricEnd() || !follower.isBusy()) {
            if(forward < 8) {
                if (forward == 1) {
                    first_score = 2;
                    if (follower.getVelocity().getMagnitude() <= 2) {
                        pic = 2;
                        forward = 9.5;
                    }

                }
            }else {
                if (forward == 9) {
                    //comeback to human player to pick up a block
                    if (follower.getVelocity().getMagnitude() <= 2) {
                        pic = 2;
                        forward = 9.5;
                    }

                } else if(forward == 9.5 && (pic == 4.5 || pic == 1)){
                    if(count == 1){
                        follower.followPath(comeback1);
                    }else if(count == 2){
                        follower.followPath(comeback2);
                    }else if(count == 3){
                        follower.followPath(comeback3);
                    }else if(count == 4){
                        follower.followPath(comeback4);
                    } else if(count == 5){
                        follower.followPath(comeback5);
                    }else if(count == 6){
                        follower.followPath(comeback6);
                    } else if(count == 7){
                        follower.followPath(comeback7);
                    }else if(count == 8){
                        follower.followPath(comeback8);
                    }
                    forward = 10;
                }else if (forward == 10) {
                    //pick block from human player
                    comeback1ish = new Path(new BezierCurve(new Point(10, -18.8, Point.CARTESIAN), new Point(6, -18.8, Point.CARTESIAN), new Point(4.5  , -18.8, Point.CARTESIAN)));
                    comeback1ish.setReversed(true);
                    follower.followPath(comeback1ish);
                    gripspinny.setPower(-1);
                    human_player = 2;
                    forward = 11;
                } else if (forward == 11) {
                    //go to score the block
                    armtarget = 732;
                    slidestarget = 648;
                    wristpose = .75;
                    twistpose = 0;
                    flippose = .65;
                    count += 1;
                    drivetime.reset();
                    if (count == 2) {
                        follower.followPath(score3);
                    } else if (count == 3) {
                        follower.followPath(score4);
                    } else if (count == 4) {
                        follower.followPath(score5);
                    } else if(count == 5){
                        follower.followPath(score6);
                    }else if(count == 6){
                        follower.followPath(score2);
                    }else if(count == 7){
                        follower.followPath(score7);
                    }else if(count == 8){
                        follower.followPath(score8);
                    }
                    if(count >= 6){
                        forward = 9.5;
                    }else {
                        forward = 9;
                    }
                }

            }
        }
        telemetry.addData("going forward", follower.isLocalizationNAN());
        telemetry.addData("going forward", follower.isRobotStuck());
        telemetry.addData("going forward", follower.atParametricEnd());
        telemetry.addData("going forward", limitwrist1.getState());
        telemetry.addData("going forward", Constants.fConstants);
        telemetry.addData("slide target", slidestarget);
        telemetry.update();
    }



}

