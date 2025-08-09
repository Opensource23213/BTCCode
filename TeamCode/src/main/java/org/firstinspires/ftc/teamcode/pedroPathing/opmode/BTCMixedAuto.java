package org.firstinspires.ftc.teamcode.pedroPathing.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@Config
@Autonomous (name = "MixedAuto", group = "AAA", preselectTeleOp = "AutoArmop"
)
public class BTCMixedAuto extends BTCLibrary {
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
        score1 = new Path(new BezierLine(new Point(0, 6, Point.CARTESIAN), new Point(35, 16.5, Point.CARTESIAN)));
        score1.setConstantHeadingInterpolation(0);
        score2 = new Path(new BezierCurve(new Point(4, -18.5, Point.CARTESIAN), new Point(9.1, 0, Point.CARTESIAN), new Point(12.3, 10, Point.CARTESIAN), new Point(23.3, 12, Point.CARTESIAN), new Point(36, 14.2, Point.CARTESIAN)));
        score2.setConstantHeadingInterpolation(0);
        push1 = new Path(new BezierCurve(new Point(36, 14.5, Point.CARTESIAN), new Point(16, 14, Point.CARTESIAN), new Point(16,0, Point.CARTESIAN), new Point(16,-30, Point.CARTESIAN), new Point(64,-25, Point.CARTESIAN), new Point(63,-26, Point.CARTESIAN), new Point(16,-42, Point.CARTESIAN)));
        push1.setConstantHeadingInterpolation(0);
        push2 = new Path(new BezierCurve(new Point(13,-42, Point.CARTESIAN), new Point(4.5,-42, Point.CARTESIAN)));
        push2.setConstantHeadingInterpolation(0);
        push3ish = new Path(new BezierCurve(new Point(3,-43.5, Point.CARTESIAN),new Point(8,-38, Point.CARTESIAN), new Point(10.3,0, Point.CARTESIAN), new Point(16.3,7, Point.CARTESIAN), new Point(25,10, Point.CARTESIAN), new Point(31,12, Point.CARTESIAN)));
        push3ish.setConstantHeadingInterpolation(0);
        score2 = new Path(new BezierCurve(new Point(5, -18.5, Point.CARTESIAN), new Point(9.1, 0, Point.CARTESIAN), new Point(12.3, 10, Point.CARTESIAN), new Point(23.3, 12, Point.CARTESIAN), new Point(35, 17, Point.CARTESIAN)));
        score2.setConstantHeadingInterpolation(0);
        comeback1 = new Path(new BezierCurve(new Point(28, 18, Point.CARTESIAN), new Point(24.5, 8, Point.CARTESIAN), new Point(18.9, -12.3, Point.CARTESIAN), new Point(11, -15.8, Point.CARTESIAN)));
        comeback1.setConstantHeadingInterpolation(0);
        comeback2 = new Path(new BezierCurve(new Point(27, 16.5, Point.CARTESIAN), new Point(14.9, -15.3, Point.CARTESIAN), new Point(10, -18.8, Point.CARTESIAN)));
        comeback2.setConstantHeadingInterpolation(0);
        comeback3 = new Path(new BezierCurve(new Point(28, 15, Point.CARTESIAN), new Point(23.5, 8, Point.CARTESIAN), new Point(18.9, -12.3, Point.CARTESIAN), new Point(11, -15.8, Point.CARTESIAN)));
        comeback3.setConstantHeadingInterpolation(0);
        comeback4 = new Path(new BezierCurve(new Point(32, 12, Point.CARTESIAN), new Point(25, 7, Point.CARTESIAN), new Point(18.9, -12.3, Point.CARTESIAN), new Point(11, -15.8, Point.CARTESIAN)));
        comeback4.setConstantHeadingInterpolation(0);
        comeback5 = new Path(new BezierCurve(new Point(32, 12, Point.CARTESIAN), new Point(25, 7, Point.CARTESIAN), new Point(18.9, -12.3, Point.CARTESIAN), new Point(11, -15.8, Point.CARTESIAN)));
        comeback5.setConstantHeadingInterpolation(0);
        comeback6 = new Path(new BezierCurve(new Point(28, 12, Point.CARTESIAN), new Point(23.5, 8, Point.CARTESIAN), new Point(18.9, -12.3, Point.CARTESIAN), new Point(11, -15.8, Point.CARTESIAN)));
        comeback6.setConstantHeadingInterpolation(0);
        comeback7 = new Path(new BezierCurve(new Point(28, 12, Point.CARTESIAN), new Point(23.5, 8, Point.CARTESIAN), new Point(18.9, -12.3, Point.CARTESIAN), new Point(11, -15.8, Point.CARTESIAN)));
        comeback7.setConstantHeadingInterpolation(0);
        comeback8 = new Path(new BezierCurve(new Point(28, 11, Point.CARTESIAN), new Point(23.5, 8, Point.CARTESIAN), new Point(18.9, -12.3, Point.CARTESIAN), new Point(11, -15.8, Point.CARTESIAN)));
        comeback8.setConstantHeadingInterpolation(0);
        comeback1ish = new Path(new BezierCurve(new Point(11, -18, Point.CARTESIAN), new Point(6, -18.5, Point.CARTESIAN), new Point(4, -18.5, Point.CARTESIAN)));
        comeback1ish.setConstantHeadingInterpolation(0);
        comeback2ish = new Path(new BezierLine(new Point(11, -17.5, Point.CARTESIAN), new Point(2, -30.5, Point.CARTESIAN)));
        comeback2ish.setConstantHeadingInterpolation(0);
        comeback2 = new Path(new BezierCurve(new Point(28, 15.4, Point.CARTESIAN), new Point(23.5, 8, Point.CARTESIAN), new Point(18.9, -12.3, Point.CARTESIAN), new Point(11, -15.8, Point.CARTESIAN)));
        comeback2.setConstantHeadingInterpolation(0);
        score3 = new Path(new BezierCurve(new Point(5, -18.5, Point.CARTESIAN), new Point(9.1, 0, Point.CARTESIAN), new Point(12.3, 10, Point.CARTESIAN), new Point(23.3, 12, Point.CARTESIAN), new Point(34, 15, Point.CARTESIAN)));
        score3.setConstantHeadingInterpolation(0);
        score4 = new Path(new BezierCurve(new Point(5, -18.5, Point.CARTESIAN), new Point(9.1, 0, Point.CARTESIAN), new Point(12.3, 10, Point.CARTESIAN), new Point(23.3, 12, Point.CARTESIAN), new Point(31, 14.5, Point.CARTESIAN)));
        score4.setConstantHeadingInterpolation(0);
        score5 = new Path(new BezierCurve(new Point(5, -18.5, Point.CARTESIAN), new Point(9.1, 0, Point.CARTESIAN), new Point(12.3, 10, Point.CARTESIAN), new Point(23.3, 12, Point.CARTESIAN), new Point(31, 13, Point.CARTESIAN)));
        score5.setConstantHeadingInterpolation(0);
        score6 = new Path(new BezierCurve(new Point(5, -18.5, Point.CARTESIAN), new Point(9.1, 0, Point.CARTESIAN), new Point(12.3, 10, Point.CARTESIAN), new Point(23.3, 14, Point.CARTESIAN), new Point(31, 14, Point.CARTESIAN)));
        score6.setConstantHeadingInterpolation(0);
        score7 = new Path(new BezierCurve(new Point(5, -18.5, Point.CARTESIAN), new Point(9.1, 0, Point.CARTESIAN), new Point(12.3, 10, Point.CARTESIAN), new Point(23.3, 14, Point.CARTESIAN), new Point(31, 14, Point.CARTESIAN)));
        score7.setConstantHeadingInterpolation(0);
        score8 = new Path(new BezierCurve(new Point(5, -18.5, Point.CARTESIAN), new Point(9.1, 0, Point.CARTESIAN), new Point(12.3, 10, Point.CARTESIAN), new Point(23.3, 12, Point.CARTESIAN), new Point(35, 12, Point.CARTESIAN)));
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
        preselect_menu();
        arm();
        telemetry.update();
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
            slidestarget = 240;
            wristpose = .62;
            twistpose = 0;
            flippose = .56;
            intake_turret_pos = .3;
            intake_vertical_pos = ready_pose;
            if(feed_machines){
                limelight.pipelineSwitch(4);
                intake_vertical_pos = .23;
                intake_turret_pos = .13;
            }
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
        feedgreen();
        intake_vertical_move();
        follower.update();
        if(one == 1) {
            if ((forward == 1 || forward == 9 || forward == 9.5 || forward == 20)) {
                if (follower.getPose().getX() > 20 && follower.getPose().getX() < 25) {
                    follower.setMaxPower(.6);
                }
            } else {
                follower.setMaxPower(1);
            }
            if (forward == 10 || forward == 11) {
                if (follower.getPose().getX() < 27) {
                    armtarget = 732;
                    wristpose = .43;
                    slidestarget = 0;
                    flippose = .025;
                    if (follower.getPose().getX() < 25 && (count < 3 || (count < 4 && seven))) {
                        intake_turret_pos = turret_drop + .025;
                        intake_vertical_pos = .22;
                        intake_twist_pos = .77;
                    }
                    if(follower.getPose().getX() <= 18){
                        if(count < 3 || (count < 4 && seven)) {
                            human_player = 2;
                        }
                    }


                    twistpose = .56;
                }
            }
            if (follower.atParametricEnd() || !follower.isBusy() || ((forward == 1 || forward == 9) && follower.getVelocity().getXComponent() <= 4 && follower.getPose().getX() > 28)) {
                if (forward < 8) {
                    if (forward == 1) {
                        first_score = 2;
                        if (follower.getVelocity().getXComponent() <= 4) {
                            follower.breakFollowing();
                            follower.holdPoint(new Pose(follower.getPose().getX() + 1, follower.getPose().getY()));
                            pic = 2;
                            if(feed_machines) {
                                multipick = 2;
                            }
                            forward = 9.5;
                            wristpose = .43;
                            slidestarget = 300;
                            flippose = .61;
                            gripspinny.setPower(1);
                        }

                    }
                } else {
                    if (forward == 9) {
                        //comeback to human player to pick up a block
                        if (follower.getVelocity().getXComponent() <= 4) {
                            follower.breakFollowing();
                            follower.holdPoint(new Pose(follower.getPose().getX() + 1, follower.getPose().getY()));
                            pic = 2;
                            forward = 9.5;
                            wristpose = .43;
                            slidestarget = 300;
                            flippose = .61;
                            gripspinny.setPower(1);
                        }

                    } else if (forward == 9.5 && (pic == 4.5 || pic == 1) && multipick == 1) {
                        if (count == 1) {
                            follower.followPath(comeback1);
                        } else if (count == 2) {
                            follower.followPath(comeback2);
                        } else if (count == 3) {
                            follower.followPath(comeback3);
                        } else if (count == 4) {
                            follower.followPath(comeback4);
                        } else if (count == 5) {
                            follower.followPath(comeback5);
                        } else if (count == 6) {
                            follower.followPath(comeback6);
                        } else if (count == 7) {
                            follower.followPath(comeback7);
                        } else if (count == 8) {
                            follower.followPath(comeback8);
                        }
                        wristpose = .43;
                        slidestarget = 300;
                        flippose = .61;
                        gripspinny.setPower(1);
                        forward = 10;
                    } else if (forward == 10) {
                        //pick block from human player
                        comeback1ish = new Path(new BezierCurve(new Point(10, -17.8, Point.CARTESIAN), new Point(8, -17.8, Point.CARTESIAN), new Point(6.25, -17.8, Point.CARTESIAN)));
                        comeback1ish.setReversed(true);
                        follower.followPath(comeback1ish);
                        if(count == 6 || (count == 7 && seven)){
                            forward = 99;
                        }else {
                            gripspinny.setPower(-1);
                            forward = 11;
                        }
                    } else if (forward == 11) {
                        //go to score the block
                        armtarget = 732;
                        slidestarget = 240;
                        wristpose = .62;
                        twistpose = 0;
                        flippose = .56;
                        intake_vertical_pos = .23;
                        intake_turret_pos = .13;
                        count += 1;
                        drivetime.reset();
                        if (count == 2) {
                            follower.followPath(score2);
                        } else if (count == 3) {
                            follower.followPath(score3);
                        } else if (count == 4) {
                            follower.followPath(score4);
                        } else if (count == 5) {
                            follower.followPath(score5);
                        } else if (count == 6) {
                            follower.followPath(score6);
                        } else if (count == 7) {
                            follower.followPath(score7);
                        } else if (count == 8) {
                            follower.followPath(score8);
                        }
                        if (seven) {
                            if (count >= 4) {
                                if (count == 4) {
                                    forward = 20;
                                } else {
                                    forward = 9.5;
                                }
                            } else {
                                forward = 9;
                            }
                        }else {
                            if (count >= 3) {
                                if (count == 3) {
                                    forward = 20;
                                } else {
                                    forward = 9.5;
                                }
                            } else {
                                forward = 9;
                            }
                        }
                    } else if (forward == 20) {
                        follower.followPath(push1);
                        intake_turret_pos = .2;
                        intake_vertical_pos = drop_pose;
                        wristpose = .43;
                        slidestarget = 300;
                        flippose = .61;
                        gripspinny.setPower(1);
                        follower.setMaxPower(1);
                        one = 2;
                        forward = 21;
                    }
                    else if (forward == 21) {
                        follower.followPath(push3ish);
                        armtarget = 732;
                        wristpose = .75;
                        twistpose = 0;
                        flippose = .65;
                        slidestarget = 648;
                        count += 1;
                        forward = 9.5;
                    }


                }
            }
        }else if(one >= 2){
            if (follower.getPose().getX() < 25) {
                armtarget = 732;
                wristpose = .43;
                slidestarget = 0;
                flippose = .025;
                twistpose = .56;
            }
            if (follower.getPose().getY() < -32 && follower.getPose().getX() < 20 && one == 2) {
                slideservopose = .18;
                intake_turret_pos = .6;
                intake_vertical_pos = .025;
                intake_twist_pos = .5;
                gripspinny.setPower(-1);
                follower.setMaxPower(1);
                one = 3;
            }

            if (follower.getVelocity().getMagnitude() < 2 && follower.getPose().getY() >= -44 && follower.getPose().getX() < 20 && one == 3) {
                pic = 2.5;
                follower.breakFollowing();
                follower.holdPoint(new Pose(follower.getPose().getX(), follower.getPose().getY()));
                one = 4;
            }
            if (gripper_at < .783 && one == 4) {
                human_player = 2;
                pic = 1;
                follower.followPath(push2);
                one = 1;
                forward = 21;
            }
        }
        telemetry.addData("going forward", follower.getPose().getX());
        telemetry.addData("going forward", follower.getPose().getY());
        telemetry.addData("going forward", follower.atParametricEnd());
        telemetry.addData("going forward", limitwrist1.getState());
        telemetry.addData("going forward", Constants.fConstants);
        telemetry.addData("slide target", slidestarget);
        telemetry.update();
    }
    public double step = 1;
    public boolean selected = false;
    public boolean six = false;
    public boolean feed_machines = false;
    public boolean seven = false;
    public void preselect_menu(){
        if(step == 1){
            telemetry.addLine("To run a six auto, press : X");
            telemetry.addLine("To run a six auto and feed green machines, press : Triangle");
            telemetry.addLine("To run a seven auto, press : Square");
            if(gamepad1.a){
                six = true;
                selected = true;
            }
            if(gamepad1.x){
                seven = true;
                selected = true;
            }
            if(gamepad1.y){
                feed_machines = true;
                selected = true;
            }
            if(gamepad1.atRest() && selected){
                step = 2;
                selected = false;
            }
        }else if(step == 2){
            telemetry.addLine("To reselect your autonomous, press : Circle");
            if(gamepad1.b){
                selected = true;
            }
            if(gamepad1.atRest() && selected){
                step = 1;
                six = false;
                seven = false;
                feed_machines = false;
                selected = false;
            }
        }
    }
    public void fastpick(){
        if(pic == 4.5 && one == 4){
            human_player = 2;
            one = 5;
        }else if(one == 5 && human_player == 1){
            pic = 2.5;
            one = 6;
        }else if(one == 6 && pic == 4.5){
            human_player = 2;
            one = 7;
        }
    }



}

