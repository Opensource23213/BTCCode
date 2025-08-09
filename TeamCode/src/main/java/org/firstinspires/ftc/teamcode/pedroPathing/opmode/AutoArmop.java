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
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.concurrent.TimeUnit;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;


@Config
@TeleOp(name="AutoArmop", group="ABC Opmode")
//@Disabled
public class AutoArmop extends OpMode {

    private PIDController controller;
    private PIDController armcontroller;
    public Follower follower;

    public DcMotorEx fl = null;
    public DcMotorEx rl = null;
    public DcMotorEx fr = null;
    public DcMotorEx rr = null;
    public DcMotorEx a1 = null;
    public DcMotorEx a2 = null;


    public static double p = 0.004, i = 0, d = 0;

    public static double f = 0.01;

    public static int slidestarget = 0;
    public static double armp = 0.01, armi = 0, armd = 0;

    public static double armf = 0.01;

    public static int armtarget = 0;


    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    public double wrist_at;
    public double armPose = 0;
    public AnalogInput wristencoder;

    private DcMotor slides = null;
    private DcMotor Arm1 = null;
    private DcMotor Arm2 = null;
    private AnalogInput ArmPos = null;
    private Servo wristy = null;
    private Servo twisty = null;
    private Servo light = null;

    double mode = 1;
    double county = 2;
    public double basketmove = 1;
    double slideratio = 2;
    double slideticks = 103.8 * slideratio / 4.75;
    double armticks = 8192 / 360;
    double toplimit = 18.6;

    double bottomlimit = .25;
    double slidebasket = 1700;
    double armbasket = 1725;
    double twistbasket = .5;
    double wristbasket = .2;
    double slidespecimen = .5;
    double armspecimen = 1380;
    double wristspecimen = .3;
    double twistspecimen = .5;
    public double tx = 0;
    double armspecimenpickup = 60;
    double wristspecimenpickup = .51;
    double xpress = 1;
    double times = 0;
    public Button1 buttons1 = null;
    public Button2 buttons2 = null;
    public double start = 0;
    public IMU imu = null;
    public DcMotor front_left = null;
    public DcMotor rear_left = null;
    public DcMotor front_right = null;
    public DcMotor rear_right = null;
    public double apress = 1;
    public double flippose = .561;
    double rpress = 1;

    public RevTouchSensor limitfront;
    public RevTouchSensor limitfront2;
    public DigitalChannel limitwrist1;
    public DigitalChannel limitslide;
    public double r1press = 1;
    double press = 1;
    double slidesPose = 0;
    double offset = 0;
    double many = 1;
    double newpos = -312;
    double ypress = 1;
    double check = 1;
    double oldangle = 0;
    double oldtarget = 0;
    double dpress = 1;
    public spin gripspinny;
    public double spit = 1;
    enum State {
        TRAJECTORY_1,   // First, follow a splineTo() trajectory
        TRAJECTORY_2,   // Then, follow a lineTo() trajectory
        // Then we want to do a point turn
        TRAJECTORY_3,   // Then, we follow another lineTo() trajectory
        TRAJECTORY_4,         // Then we're gonna wait a second
        TRAJECTORY_5,         // Finally, we're gonna turn again
        TRAJECTORY_6,
        IDLE            // Our bot will enter the IDLE state when done
    }

    // We define the current state we're on
    // Default to IDLE
    double front = 0;
    public double dropping = 1;
    public double special_pick = 1;
    double scored = 1;

    // Define our start pose
    // This assumes we start at x: 15, y: 10, heading: 170 degrees
    RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
    RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;

    RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
    double just = 0;
    boolean ready = false;
    double aapress = 1;
    ElapsedTime drivetime = new ElapsedTime();
    double ticks = .002866;
    double conversion = ticks * armticks;
    public double inta = 1;
    public flippy flip;
    double wristpose = .5;
    double twistpose = 0;
    double lasttraj = 0;
    double safety = 1;
    double flipsafe = 1;
    boolean auto = false;
    private Path score3;
    private Path score3ish;
    private Path score4;
    private Path score4ish;
    private Path score5;
    private Path score5ish;
    private Path comeback1;
    private Path comeback1ish;
    public double forward = 5;
    public double start_auto = 1;
    public double a = 1;
    public static double spit_time = 150;
    boolean slidelimit = true;
    public double yypress = 1;
    public double downish = 1;
    public double scoring = 1;
    public double stick = 1;
    public boolean locked = false;
    Limelight3A limelight;
    public ElapsedTime flasher = new ElapsedTime();
    public double good = 1;
    public double flmax = 0;
    public double rlmax = 0;
    public double frmax = 0;
    public double rrmax = 0;
    public double totalmax = 0;
    public boolean takeover = false;
    public boolean color = true;
    public double hope = 0;
    public RevColorSensorV3 side = null;
    public boolean first_read = true;
    public double lowpress = 1;
    public double r1count = 0;
    public double switchcount = 0;
    public boolean stop = false;
    public double oldtick = 0;
    public double feed = 1;


    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        controller = new PIDController(p, i, d);
        armcontroller = new PIDController(armp, armi, armd);
        gripspinny = new spin();
        gripspinny.initialize();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        slides = hardwareMap.get(DcMotor.class, "slides"); //0 to -3.5 limit
        Arm1 = hardwareMap.get(DcMotor.class, "Arm1");
        Arm2 = hardwareMap.get(DcMotor.class, "Arm2");
        a1 = hardwareMap.get(DcMotorEx.class, "Arm1");
        a2 = hardwareMap.get(DcMotorEx.class, "Arm2");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        ArmPos = hardwareMap.get(AnalogInput.class, "ArmPos");
        wristy = hardwareMap.get(Servo.class, "wrist");
        twisty = hardwareMap.get(Servo.class, "twist");
        fl = hardwareMap.get(DcMotorEx.class, "front_left");
        fr = hardwareMap.get(DcMotorEx.class, "front_right");
        rl = hardwareMap.get(DcMotorEx.class, "rear_left");
        rr = hardwareMap.get(DcMotorEx.class, "rear_right");
        front_left = hardwareMap.get(DcMotor.class, "front_left");
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        rear_left = hardwareMap.get(DcMotor.class, "rear_left");
        rear_right = hardwareMap.get(DcMotor.class, "rear_right");
        limitwrist1 = hardwareMap.get(DigitalChannel.class, "limitwrist1");
        limitslide = hardwareMap.get(DigitalChannel.class, "limitslide");
        limitfront = hardwareMap.get(RevTouchSensor.class, "limitfront");
        limitfront2 = hardwareMap.get(RevTouchSensor.class, "limitfront2");
        wristencoder = hardwareMap.get(AnalogInput.class, "wristencoder");
        side = hardwareMap.get(RevColorSensorV3.class, "color");

        light = hardwareMap.get(Servo.class, "light");
        limelight.setPollRateHz(150); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
        limelight.pipelineSwitch(4);
        comeback1 = new Path(new BezierCurve(new Point(26.8, 19, Point.CARTESIAN), new Point(20.2, 2.8, Point.CARTESIAN), new Point(16.9, -12.3, Point.CARTESIAN), new Point(10, -16.8, Point.CARTESIAN)));
        comeback1.setConstantHeadingInterpolation(0);
        comeback1ish = new Path(new BezierLine(new Point(10, -16.8, Point.CARTESIAN), new Point(4.1, -16.9, Point.CARTESIAN)));
        comeback1ish.setConstantHeadingInterpolation(0);
        score3 = new Path(new BezierCurve(new Point(6.1, -14.9, Point.CARTESIAN), new Point(12.1, .7, Point.CARTESIAN), new Point(17.3, 11.4, Point.CARTESIAN), new Point(27.5, 11.1, Point.CARTESIAN)));
        score3.setConstantHeadingInterpolation(0);
        score3ish = new Path(new BezierLine(new Point(27, 11.6, Point.CARTESIAN), new Point(31, 15.4, Point.CARTESIAN)));
        score3ish.setConstantHeadingInterpolation(0);
        gripspinny.setPower(0);
        slides.setDirection(DcMotor.Direction.FORWARD);
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Arm1.setDirection(DcMotor.Direction.REVERSE);
        Arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_left.setDirection(DcMotor.Direction.REVERSE);
        front_right.setDirection(DcMotor.Direction.FORWARD);
        rear_left.setDirection(DcMotor.Direction.REVERSE);
        rear_right.setDirection(DcMotor.Direction.FORWARD);
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rear_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rear_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        buttons1 = new Button1();
        buttons2 = new Button2();
        flip = new flippy();
        // Define a 1.5 second wait time
        double waitTime1 = 1.5;
        ElapsedTime waitTimer1 = new ElapsedTime();
        flip.initialize();
        follower.setPose(new Pose(0,0,0));
        follower.setHeadingOffset(Math.toRadians(0));
        // Define the angle for turn 2
        double turnAngle2 = Math.toRadians(720);
        boolean ready = false;
        armtarget = (int) ((1 - ArmPos.getVoltage() - .2) / ticks * armticks);
        slidestarget = 0;
        gamepad2.setLedColor(255,0,255,999999999);
        gamepad1.setLedColor(255,162,0,999999999);
    }



    @Override
    public void loop() {
        first();
        buttons2.button();
        drive();
        arm();
        autotwist();
        drop();
        spit();
        extra_in();
        basket();
        dropoff();
        dropoff2();
        eddy();
        basketdrop();
        spit2();
        check();
        safeflip();
        servosafe();
        buttons1.button();
        release();
        gripspinny.onetwo();
        longscore();
        getwall();

        newspit();
        lowscore();
    }
    public void first(){
        if(first_read){
            double what = side.blue();
            if(what > 5000){
                telemetry.addData("Color = ", "Blue");
                limelight.pipelineSwitch(7);
            }else{
                telemetry.addData("Color = ", "Red");
                limelight.pipelineSwitch(8);
            }
            first_read = false;
        }
    }
    public void arm(){
        toplimit = (7 * slideticks * 2);
        wrist_at = abs(1 - wristencoder.getVoltage() / 3.3) + .013;
        controller.setPID(p, i, d);
        slidesPose = -slides.getCurrentPosition() * 2;
        armd = (-slides.getCurrentPosition()) / slideticks * .03 / 19.6;
        armf = .001 + (-slides.getCurrentPosition()) / slideticks * .2 / 19;
        double pid = controller.calculate(slidesPose, slidestarget);
        double ff = Math.cos(Math.toRadians(slidestarget)) * f;
        double power = pid + ff;
        if(slidestarget > 1700 && buttons1.lastbutton == "r1"){
            if(slidesPose > 1700){
                power = -.3;
                slides.setPower(power);
            }else{
                power = -1;
                slides.setPower(power);
            }

        }
        else if (((-350 < slidesPose - slidestarget && slidesPose - slidestarget < 350) || buttons2.lastbutton == "x") && ((gamepad2.right_stick_y > .1 || gamepad2.right_stick_y < -.1 || buttons2.lastbutton == "x"))&& !locked) {
            double otherPose = -slides.getCurrentPosition() / slideticks;
            if (!gamepad2.ps && otherPose < bottomlimit && gamepad2.right_stick_y > 0) {
                slides.setPower(0);
                slidestarget = (int) (-slides.getCurrentPosition() * 2);
            } else if (slidesPose > toplimit && gamepad2.right_stick_y < 0) {
                slides.setPower(0);
                slidestarget = (int) (-slides.getCurrentPosition() * 2);
            } else {
                slides.setPower(gamepad2.right_stick_y * .8);
                slidestarget = (int) (-slides.getCurrentPosition() * 2);
            }
        } else if(a == 2){
            if(slidestarget == 0 && slidesPose < 10 && limitslide.getState()){
                slides.setPower(.4);
            }else {
                if(!limitslide.getState() && slidelimit && slidestarget < 20){
                    slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    slides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    slidestarget = 1;
                    slidelimit = false;
                }else if(limitslide.getState()){
                    slidelimit = true;
                }
                if(buttons2.lastbutton != "x") {
                    slides.setPower(-power);
                }
            }

        }
        armcontroller.setPID(armp, armi, armd);
        armPose = (1 - ArmPos.getVoltage() - .2) / ticks * armticks;
        double armpid = controller.calculate(armPose, armtarget);
        double armff = Math.cos(Math.toRadians(armtarget)) * armf;
        double armpower = armpid + armff;
        if(buttons2.lastbutton == "l32"){
            if(armPose < 30){
                armtarget = 0;
                buttons2.lastbutton = "";
            }else{
                armpower = -1;
                Arm1.setPower(-armpower);
                Arm2.setPower(-armpower);
            }

        }
        else if (abs(abs(armPose) - abs(armtarget)) < 150 && ((gamepad2.left_stick_y > .1 || gamepad2.left_stick_y < -.1) && !auto) && buttons2.lastbutton != "a") {
            if (armPose < newpos + 45 && gamepad2.left_stick_y > 0) {
                Arm1.setPower(0);
                Arm2.setPower(0);
                armtarget = (int) (armPose);
            } else if(dpress == 2){
                Arm1.setPower(-armpower);
                Arm2.setPower(-armpower);
            }else {
                Arm1.setPower(gamepad2.left_stick_y / 2);
                Arm2.setPower(gamepad2.left_stick_y / 2);
                armtarget = (int) (armPose);
            }
        } else {
            Arm1.setPower(-armpower);
            Arm2.setPower(-armpower);
        }
        telemetry.addData("switch", switchcount);
        telemetry.addData("r1", r1count);
        telemetry.addData("pose", armPose/armticks);
        telemetry.addData("target", armtarget);
        telemetry.addData("power", -armpower);
        telemetry.addData("now - target", slidesPose - slidestarget);
        telemetry.addData("pose", slidesPose);
        telemetry.addData("target", slidestarget);
        telemetry.addData("power", -power);
        telemetry.addData("Limit1", limitwrist1.getState());
        telemetry.addData("Limitslide", limitslide.getState());
        telemetry.addData("Flippy position", flip.getPosition());
        telemetry.addData("Wrist position", wrist_at);
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("X", Math.toDegrees(follower.getPose().getX()));
        telemetry.addData("Y", Math.toDegrees(follower.getPose().getY()));
        telemetry.addData("Front Left Current", fl.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Rear Left Current", rl.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Front Right Current", fr.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Rear Right Current", rr.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Front Left Max Current", flmax);
        telemetry.addData("Rear Left Max Current", rlmax);
        telemetry.addData("Front Right Max Current", frmax);
        telemetry.addData("Rear Right Max Current", rrmax);
        telemetry.addData("Total Max Current", totalmax);
        telemetry.addData("Arm 1 Current", a1.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Arm 2 Current", a2.getCurrent(CurrentUnit.AMPS));
        telemetry.update();
        flmax = maxVoltage(fl.getCurrent(CurrentUnit.AMPS), flmax);
        frmax = maxVoltage(fr.getCurrent(CurrentUnit.AMPS), flmax);
        rlmax = maxVoltage(rl.getCurrent(CurrentUnit.AMPS), flmax);
        rrmax = maxVoltage(rr.getCurrent(CurrentUnit.AMPS), flmax);
        maxTotalVoltage();
        if(a == 2) {
            wristy.setPosition(wristpose - .04);
            twisty.setPosition(twistpose + .028);
            flip.setPosition(flippose);
        }
    }
    public class Button2{
        String button = "";
        String nowbutton = "";
        String lastbutton = "";
        String type = "";
        public void button(){
            if (button == "") {
                if (gamepad2.left_stick_y > .2){
                    button = "lsy";
                }
                else if (gamepad2.left_stick_y < -.3){
                    button = "lsyu";
                }
                else if (gamepad2.a) {
                    button = "a";
                }
                else if(gamepad2.b){
                    button = "b";
                }
                else if (gamepad2.x){
                    button = "x";
                }
                else if (gamepad2.y){
                    button = "y";
                }
                else if (gamepad2.right_bumper){
                    button = "r1";
                }
                else if (gamepad2.left_bumper){
                    button = "l1";
                }
                else if (gamepad2.left_trigger > .4){
                    button = "l2";
                }
                else if (gamepad2.right_trigger > .4){
                    button = "r2";
                }else if (gamepad2.dpad_up){
                    button = "up";
                }
                else if (gamepad2.dpad_down){
                    button = "down";
                }
                else if (gamepad2.dpad_left){
                    button = "left";
                }
                else if (gamepad2.dpad_right){
                    button = "right";
                }else if (gamepad2.ps){
                    button = "ps";
                }
                else if (gamepad2.left_stick_button){
                    button = "l3";
                }
                else if (gamepad2.right_stick_button){
                    button = "r3";
                }else if (gamepad2.touchpad){
                    button = "middle";
                }
            }
            endbutton();
            ButtonControl();
            nowbutton = "";
        }
        public void ButtonControl(){
            if(nowbutton == "ps"){
                slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                slidestarget = 0;
                nowbutton = "";
            }
            if(nowbutton == "r2"){
                a = 2;
                light.setPosition(0);
                feed = 1;
                wristpose = .5;
                twistpose = 0;
                armtarget = 0;
                slidestarget = 0;
                flippose = .561;
                takeover = false;
                safety = 2;
                locked = false;
                gripspinny.setPower(0);
                lastbutton = "";
                nowbutton = "";
                dpress = 1;
            }
            if(lastbutton == ""){
                if (nowbutton == "l3") {
                    armtarget = 2210;
                    wristpose = .34;
                    twistpose = 0;
                    slidestarget = 0;
                    flippose = .583;
                    light.setPosition(0);
                    lastbutton = "l3";
                    nowbutton = "";
                    dpress = 1;
                }
                else if(nowbutton == "down"){
                    armtarget = (int) 1412;
                    slidestarget = (int) 650;
                    wristpose = .72;
                    flippose = .44;
                    twistpose = 0.56;
                    light.setPosition(0);
                    lastbutton = "down";
                    nowbutton = "";
                    dpress = 1;
                }
                else if(nowbutton == "a" && aapress == 1 && spit == 1){
                    //Arm goes out
                    armtarget = 0;
                    light.setPosition(1);
                    mode = 1;
                    a = 2;
                    slidestarget = (int) (2 * slideticks * 2);
                    wristpose = .33;
                    twistpose = 0;
                    flippose = .56;
                    apress = 1;
                    gripspinny.setPower(-1);
                    downish = 1;
                    lastbutton = "a";
                    nowbutton = "";
                    dpress = 1;
                }
                if(nowbutton == "b"){
                    //Spit out
                    ypress = 1.5;
                    lastbutton = "";
                    nowbutton = "";
                    dpress = 1;
                }
                else if(nowbutton == "x" && aapress == 1 && spit == 1){
                    //Arm goes out
                    armtarget = 0;
                    a = 2;
                    slidestarget = (int) (2 * slideticks * 2);
                    wristpose = .33;
                    twistpose = 0;
                    flippose = .62;
                    gripspinny.setPower(-1);
                    light.setPosition(1);
                    downish = 1;
                    lastbutton = "x";
                    nowbutton = "";
                    dpress = 1;
                }

                /*else if (nowbutton == "l1"){
                    //Arm moves to hang specimen
                    armtarget = 732;
                    slidestarget = 648;
                    a = 2;
                    wristpose = .69;
                    twistpose = 0;
                    flippose = .651;
                    lastbutton = "l1";
                    nowbutton = "";
                    dpress = 1;
                }*/
                else if(nowbutton == "r1"){
                    //Arm moves to pick up stuff from eddy
                    armtarget = 732;
                    wristpose = .43;
                    slidestarget = 0;
                    flippose = .025;
                    light.setPosition(0);
                    a = 2;
                    flipsafe = 2;
                    gripspinny.setPower(-1);
                    lastbutton = "r1";
                    nowbutton = "";
                    dpress = 1;
                }
                else if(nowbutton == "y"){
                    //Arm moves to pick up stuff from eddy
                    armtarget = 732;
                    wristpose = .43;
                    light.setPosition(0);
                    slidestarget = 0;
                    flippose = .025;
                    twistpose = 0;
                    a = 2;
                    gripspinny.setPower(-1);
                    lastbutton = "y";
                    nowbutton = "";
                    dpress = 1;
                }else if(nowbutton == "middle" && color){
                    limelight.pipelineSwitch(4);
                    gamepad2.setLedColor(255,162,0,999999999);
                    color = false;
                    nowbutton = "";
                }else if(nowbutton == "middle" && !color){
                    first_read = true;
                    gamepad2.setLedColor(255,0,255,999999999);
                    color = true;
                    nowbutton = "";
                }
            }
            else if(lastbutton == "l3"){
                if(nowbutton == "l3"){
                    lastbutton = "l32";
                    nowbutton = "";
                }

            }
            else if(lastbutton == "x"){
                if(!limitwrist1.getState()){
                    //brings arm back
                    if(feed == 2){
                        wristpose = .33;
                        twistpose = 0;
                        flippose = .56;
                        spit = 2;
                        feed = 3;
                    }else if(feed == 1){
                        wristpose = .281;
                        twistpose = 0;
                        armtarget = 0;
                        light.setPosition(0);
                        oldtarget = slidesPose;
                        slidestarget = 0;
                        flippose = .561;
                        dpress = 2;
                        spit = 2;
                        nowbutton = "";
                        lastbutton = "";
                    }
                } else if(nowbutton == "a"){
                    //brings arm back
                    if(feed == 3){
                        armtarget = 0;
                        light.setPosition(1);
                        mode = 1;
                        wristpose = .33;
                        twistpose = 0;
                        flippose = .56;
                        apress = 1;
                        spit = 1;
                        gripspinny.setPower(-1);
                        downish = 1;
                        lastbutton = "x";
                        nowbutton = "";
                        dpress = 1;
                    }else {
                        wristpose = .281;
                        twistpose = 0;
                        light.setPosition(0);
                        armtarget = 0;
                        oldtarget = slidesPose;
                        slidestarget = 0;
                        flippose = .561;
                        spit = 2;
                        nowbutton = "";
                        lastbutton = "";
                    }
                }
                else if(nowbutton == "l1"){
                    flippose = .692;
                    armtarget = 0;
                    feed = 1;
                    gripspinny.setPower(-1);
                    wristpose = .31;
                    nowbutton = "";
                }
                if(nowbutton == "l2"){
                    flippose = .692;
                    armtarget = 0;
                    wristpose = .30;
                    downish = 2;
                    feed = 2;
                    locked = false;
                    nowbutton = "";
                }
                if(nowbutton == "lsyu"){
                    wristpose = .281;
                    flippose = .613;
                    downish = 1;
                    nowbutton = "";
                }
                else if(nowbutton == "right"){
                    //twists right
                    twistpose = 0;
                    nowbutton = "";
                }
                else if (nowbutton == "left"){
                    //twists left
                    twistpose = .28;
                    nowbutton = "";
                }
                else if(nowbutton == "b"){
                    //reverse intake
                    apress = 2;
                    dpress = 1;
                    nowbutton = "";
                }

            }
            else if(lastbutton == "y"){
                if(nowbutton == "y" || !limitwrist1.getState()){
                    //raise arm to take off hook and bring arm in
                    armtarget = (int) 1742;
                    flippose = .54;
                    wristpose = .7;
                    basketmove = 2;
                    twistpose = .56;
                    light.setPosition(0);
                    buttons1.lastbutton = "r1";
                    nowbutton = "";
                    dpress = 1;
                }
            }
            else if(lastbutton == "r1"){
                if((nowbutton == "r1" || !limitwrist1.getState()) && yypress == 1){
                    armtarget = 732;
                    slidestarget = 648;
                    stick = 2;
                    wristpose = .25;
                    twistpose = 0;
                    flippose = .651;
                    gripspinny.setPower(-1);
                    lastbutton = "l1";
                    nowbutton = "";
                    dpress = 1;
                }
            }
            else if(lastbutton == "down"){
                if(nowbutton == "down") {
                    lowpress = 2;
                    gripspinny.setPower(1);
                    lastbutton = "";
                    nowbutton = "";
                }
            }
            else if(lastbutton == "a"){
                if(!limitwrist1.getState()){
                    //brings arm back
                    if(feed == 2){
                        wristpose = .33;
                        twistpose = 0;
                        flippose = .56;
                        spit = 2;
                        feed = 3;
                    }else if(feed == 1){
                        wristpose = .281;
                        twistpose = 0;
                        armtarget = 0;
                        oldtarget = slidesPose;
                        slidestarget = 0;
                        light.setPosition(0);
                        flippose = .561;
                        dpress = 2;
                        spit = 2;
                        downish = 1;
                        locked = false;
                        nowbutton = "";
                        lastbutton = "";
                    }
                } else if(nowbutton == "x" && aapress == 1 && spit == 1){
                    //Arm goes out
                    armtarget = 0;
                    a = 2;
                    wristpose = .33;
                    twistpose = 0;
                    flippose = .62;
                    gripspinny.setPower(-1);
                    downish = 1;
                    lastbutton = "x";
                    nowbutton = "";
                    dpress = 1;
                }else if(nowbutton == "a"){
                    //brings arm back
                    if(feed == 3){
                        armtarget = 0;
                        light.setPosition(1);
                        mode = 1;
                        wristpose = .33;
                        twistpose = 0;
                        flippose = .56;
                        apress = 1;
                        spit = 1;
                        gripspinny.setPower(-1);
                        downish = 1;
                        lastbutton = "a";
                        nowbutton = "";
                        dpress = 1;
                    }else {
                        wristpose = .281;
                        light.setPosition(0);
                        twistpose = 0;
                        armtarget = 0;
                        oldtarget = slidesPose;
                        slidestarget = 0;
                        flippose = .561;
                        spit = 2;
                        downish = 1;
                        locked = false;
                        nowbutton = "";
                        lastbutton = "";
                    }
                }
                if(nowbutton == "l1"){
                    flippose = .692;
                    armtarget = 0;
                    wristpose = .30;
                    gripspinny.setPower(-1);
                    feed = 1;
                    downish = 2;
                    locked = false;
                    nowbutton = "";
                }
                if(nowbutton == "l2"){
                    flippose = .692;
                    armtarget = 0;
                    wristpose = .30;
                    downish = 2;
                    feed = 2;
                    locked = false;
                    nowbutton = "";
                }
                if(nowbutton == "lsyu"){
                    wristpose = .281;
                    flippose = .56;
                    downish = 1;
                    nowbutton = "";
                }
                else if(nowbutton == "right"){
                    //twists right
                    twistpose = 0;
                    nowbutton = "";
                }
                else if (nowbutton == "left"){
                    //twists left
                    twistpose = .28;
                    nowbutton = "";
                }
                else if(nowbutton == "b"){
                    //reverse intake
                    apress = 2;
                    dpress = 1;
                    nowbutton = "";
                }

            }

            else if(lastbutton == "l1"){
                if (nowbutton == "l1") {
                    //Arm drops block on the hang and goes back in
                    wristpose = .5;
                    slidestarget = 0;
                    gripspinny.setPower(1);
                    lastbutton = "";
                    nowbutton = "";

                }
                else if(nowbutton == "r1"){
                    armtarget = 732; 
                    wristpose = .43;
                    slidestarget = 0;
                    flippose = .025;
                    a = 2;
                    flipsafe = 2;
                    yypress = 1.5;
                    lastbutton = "r1";
                    nowbutton = "";
                    dpress = 1;
                }

            }


        }
        public void endbutton(){
            if (!gamepad2.a && button == "a") {
                nowbutton = "a";
                button = "";
            }
            else if(!gamepad2.b && button == "b"){
                nowbutton = "b";
                button = "";
            }
            else if (!gamepad2.x && button == "x"){
                nowbutton = "x";
                button = "";
            }
            else if (!gamepad2.y && button == "y"){
                nowbutton = "y";
                button = "";
            }
            else if (!gamepad2.right_bumper && button == "r1"){
                nowbutton = "r1";
                button = "";
            }
            else if (!gamepad2.left_bumper && button == "l1"){
                nowbutton = "l1";
                button = "";
            }
            else if (gamepad2.left_trigger < .4 && button == "l2"){
                nowbutton = "l2";
                button = "";
            }
            else if (gamepad2.right_trigger < .4 && button == "r2"){
                nowbutton = "r2";
                button = "";
            }else if (!gamepad2.dpad_up && button == "up"){
                nowbutton = "up";
                button = "";
            }
            else if (!gamepad2.dpad_down && button == "down"){
                nowbutton = "down";
                button = "";
            }
            else if (!gamepad2.dpad_left && button == "left"){
                nowbutton = "left";
                button = "";
            }
            else if (!gamepad2.dpad_right && button == "right"){
                nowbutton = "right";
                button = "";
            }else if (!gamepad2.ps && button == "ps"){
                nowbutton = "ps";
                button = "";
            }
            else if (!gamepad2.left_stick_button && button == "l3"){
                nowbutton = "l3";
                button = "";
            }
            else if (!gamepad2.right_stick_button && button == "r3"){
                nowbutton = "r3";
                button = "";
            } else if (gamepad2.left_stick_y < .2 && button == "lsy"){
                nowbutton = "lsy";
                button = "";
            }else if (gamepad2.left_stick_y > -.3 && button == "lsyu"){
                nowbutton = "lsyu";
                button = "";
            }else if (!gamepad2.touchpad && button == "middle"){
                nowbutton = "middle";
                button = "";
            }
        }
    }
    public class Button1{
        String button = "";
        String nowbutton = "";
        String lastbutton = "";
        String type = "";
        public void button(){
            if (button == "") {
                if (gamepad1.a) {
                    button = "a";
                }
                else if(gamepad1.b){
                    button = "b";
                }
                else if (gamepad1.x){
                    button = "x";
                }
                else if (gamepad1.y){
                    button = "y";
                }
                else if (gamepad1.right_bumper){
                    button = "r1";
                }
                else if (gamepad1.left_bumper){
                    button = "l1";
                }
                else if (gamepad1.left_trigger > .4){
                    button = "l2";
                }
                else if (gamepad1.right_trigger > .4){
                    button = "r2";
                }else if (gamepad1.dpad_up){
                    button = "up";
                }
                else if (gamepad1.dpad_down){
                    button = "down";
                }
                else if (gamepad1.dpad_left){
                    button = "left";
                }
                else if (gamepad1.dpad_right){
                    button = "right";
                }else if (gamepad1.ps){
                    button = "ps";
                }
                else if (gamepad1.left_stick_button){
                    button = "l3";
                }
                else if (gamepad1.right_stick_button){
                    button = "r3";
                }
            }
            endbutton();
            ButtonControl();
            nowbutton = "";
        }
        public void ButtonControl(){
            if(lastbutton == "") {
                if (nowbutton == "l2") {
                    //Arm drops block on the hang and goes back in
                    if(buttons2.lastbutton != "l1"){
                        apress = 2;
                        dpress = 1;
                    }else {
                        buttons2.nowbutton = "l1";
                    }
                    nowbutton = "";
                }
                if (nowbutton == "r2") {
                    press = 2;
                    buttons2.lastbutton = "";
                    nowbutton = "";
                    lastbutton = "";
                    dpress = 1;
                }
                if(nowbutton == "r1"){
                    armtarget = (int) 1742;
                    flippose = .54;
                    wristpose = .66;
                    basketmove = 2;
                    twistpose = .56;
                    light.setPosition(0);
                    lastbutton = "r1";
                    nowbutton = "";
                    dpress = 1;
                }
            }
            else if(lastbutton == "r1"){
                if(nowbutton == "r1") {
                    rpress = 1.5;
                    lastbutton = "";
                    nowbutton = "";
                }
            }




        }
        public void endbutton(){
            if (!gamepad1.a && button == "a") {
                nowbutton = "a";
                button = "";
            }
            else if(!gamepad1.b && button == "b"){
                nowbutton = "b";
                button = "";
            }
            else if (!gamepad1.x && button == "x"){
                nowbutton = "x";
                button = "";
            }
            else if (!gamepad1.y && button == "y"){
                nowbutton = "y";
                button = "";
            }
            else if (!gamepad1.right_bumper && button == "r1"){
                nowbutton = "r1";
                button = "";
            }
            else if (!gamepad1.left_bumper && button == "l1"){
                nowbutton = "l1";
                button = "";
            }
            else if (gamepad1.left_trigger < .4 && button == "l2"){
                nowbutton = "l2";
                button = "";
            }
            else if (gamepad1.right_trigger < .4 && button == "r2"){
                nowbutton = "r2";
                button = "";
            }else if (!gamepad1.dpad_up && button == "up"){
                nowbutton = "up";
                button = "";
            }
            else if (!gamepad1.dpad_down && button == "down"){
                nowbutton = "down";
                button = "";
            }
            else if (!gamepad1.dpad_left && button == "left"){
                nowbutton = "left";
                button = "";
            }
            else if (!gamepad1.dpad_right && button == "right"){
                nowbutton = "right";
                button = "";
            }else if (!gamepad1.ps && button == "ps"){
                nowbutton = "ps";
                button = "";
            }
            else if (!gamepad1.left_stick_button && button == "l3"){
                nowbutton = "l3";
                button = "";
            }
            else if (!gamepad1.right_stick_button && button == "r3"){
                nowbutton = "r3";
                button = "";
            }
        }
    }
    public void check(){
        if(dpress == 2 && slidesPose < 100){
            if(!limitwrist1.getState()){
                dpress = 1;
            }else{
                dpress = 3;
            }
        }
    }
    public void extra_in(){
        if(r1press == 2){
            runtime = new ElapsedTime();
            r1press = 3;
        }else if(r1press == 3 && runtime.time(TimeUnit.MILLISECONDS) < 200){
            gripspinny.setPower(-1);
        }else if(r1press == 3){
            gripspinny.setPower(0);
            r1press = 1;
        }
    }
    public void servosafe(){
        if(safety == 2 && abs(flip.getPosition() - flippose) < .04){
            wristpose = .281;
            safety = 1;
        }
    }
    public void safeflip(){
        if(flipsafe == 2 && flip.getPosition() < .45){
            twistpose = .56;
            flipsafe = 1;
        }
    }
    public void drop(){
        if(xpress == 1.5){
            runtime = new ElapsedTime();
            xpress = 2;
        }else if(xpress == 2 && runtime.time(TimeUnit.MILLISECONDS) > 750){
            gripspinny.setPower(0);
            xpress = 1;
        }else if(xpress == 2 && runtime.time(TimeUnit.MILLISECONDS) > 250){
            gripspinny.setPower(1);
        }
    }
    public void basket(){
        if(basketmove == 2 && abs(armtarget - armPose) < 50){
            slidestarget = 1700;
            basketmove = 1;
        }
    }

    public void spit(){
        if(apress == 2){
            runtime = new ElapsedTime();
            apress = 3;
        }else if(apress == 3 && runtime.time(TimeUnit.MILLISECONDS) < 320){
            gripspinny.setPower(1);
        }else if(apress == 3 && runtime.time(TimeUnit.MILLISECONDS) < 650){
            gripspinny.setPower(-1);
            apress = 1;
        }
    }
    public void spit2() {
        if (aapress == 2) {
            runtime = new ElapsedTime();
            aapress = 3;
        } else if (aapress == 3 && runtime.time(TimeUnit.MILLISECONDS) < 100) {
            gripspinny.setPower(1);
        } else if (aapress == 3 && runtime.time(TimeUnit.MILLISECONDS) < 500) {
            gripspinny.setPower(-1);
        } else if (aapress == 3) {
            gripspinny.setPower(0);
            aapress = 1;
        }
    }
    public void dropoff(){
        if(ypress == 1.5){
            runtime = new ElapsedTime();
            gripspinny.setPower(1);
            ypress = 2;
        }else if(ypress == 2 && runtime.time(TimeUnit.MILLISECONDS) > 300){
            gripspinny.setPower(-1);
            ypress = 1;
        }

    }
    public void dropoff2(){
        if(yypress == 1.5 && abs(wrist_at - wristpose) < .04){
            runtime = new ElapsedTime();
            gripspinny.setPower(.1);
            yypress = 2;
        }else if(yypress == 2 && runtime.time(TimeUnit.MILLISECONDS) > 400){
            gripspinny.setPower(-1);
            yypress = 1;
        }

    }
    public void basketdrop(){
        if(rpress == 1.5){
            if(abs(flip.getPosition() - flippose) < .05){
                rpress = 1.75;
            }
        } else if(rpress == 1.75){
            runtime = new ElapsedTime();
            gripspinny.setPower(.15);
            wristpose = .75;
            rpress = 2;
        }else if(rpress == 2 && runtime.time(TimeUnit.MILLISECONDS) > 600){
            gripspinny.setPower(0);
            flippose = .561;
            wristpose = .281;
            twistpose = 0;
            rpress = 2.5;
        }else if(rpress == 2.5){
            if(abs(wrist_at - wristpose) < .04){
                rpress = 3;
            }
        } else if(rpress == 3){
            slidestarget = 0;
            armtarget = 0;
            rpress = 1;
        }

    }
    public void eddy(){
        if(press == 2){
            armtarget = 0;
            slidestarget = 1608;
            wristpose = .221;
            flippose = .571;
            twistpose = 0;
            press = 3;
        }else if(press == 3 && abs(abs(slidesPose) - abs(slidestarget)) < 300){
            ypress = 1.5;
            press = 4;
        }else if(press == 4 && ypress == 1){
            armtarget = 0;
            slidestarget = 0;
            flippose = .561;
            wristpose = .281;
            press = 1;
        }
    }
    public class spin{
        public CRServo spinny1;
        public CRServo spinny2;
        public void initialize(){
            spinny1 = hardwareMap.get(CRServo.class, "spinny1");
            spinny2 = hardwareMap.get(CRServo.class, "spinny2");
            spinny2.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        public void onetwo(){
            if(spit == 2) {
                drivetime.reset();
                spit = 2.5;
            }else if(spit == 2.5 && drivetime.time(TimeUnit.MILLISECONDS) > spit_time/2){
                spinny1.setPower(-1);
                spinny2.setPower(1);
                drivetime.reset();
                spit = 3;
            }else if(spit == 3 && drivetime.time(TimeUnit.MILLISECONDS) > spit_time){
                drivetime.reset();
                spinny1.setPower(1);
                spinny2.setPower(-1);
                spit = 4;
            }else if(spit == 4 && drivetime.time(TimeUnit.MILLISECONDS) > spit_time){
                spit = 5;
                spinny1.setPower(-1);
                spinny2.setPower(-1);
            }
            else if(spit == 5 && drivetime.time(TimeUnit.MILLISECONDS) > 500){
                spit = 1;
                spinny1.setPower(0);
                spinny2.setPower(0);
            }
        }
        public void setPower(double power){
            spinny1.setPower(power);
            spinny2.setPower(power);
        }

        public double getPower(){
            return spinny1.getPower();
        }
    }


    public class flippy{
        public Servo flippy1;
        public Servo flippy2;
        AnalogInput flipencoder;
        public void initialize(){
            flippy1 = hardwareMap.get(Servo.class, "flippy1");
            flippy2 = hardwareMap.get(Servo.class, "flippy2");
            flipencoder = hardwareMap.get(AnalogInput.class, "flipencoder");
        }
        public void setPosition(double pos){
            flippy1.setPosition(pos);
            flippy2.setPosition(pos);
        }

        public double getPosition(){
            return abs(1 - flipencoder.getVoltage() / 3.3) + .02;
        }
    }
    public void release(){
        //Waits after dropping the block on the bar and then goes to pick
        if(dropping == 2 && slidesPose < 10) {
            if(special_pick == 2){
                armtarget = 732;
                wristpose = .46;
                slidestarget = 0;
                flippose = .025;
                dropping = 1;
                special_pick = 1;
            }else {
                armtarget = 732;
                wristpose = .43;
                slidestarget = 0;
                flippose = .025;
                flipsafe = 2;
                dropping = 1;
            }
        }
    }

    public void drive(){
        follower.update();
        Pose poseEstimate = follower.getPose();
        double angle = poseEstimate.getHeading();
        double axial = 0;
        double lateral = 0;
        double yaw = 0;
        if (gamepad1.dpad_right) { // strafe left
            axial = 0;
            lateral = .5;
            yaw = 0;
        } else if (gamepad1.dpad_left) { // strafe right
            axial = 0;
            lateral = -.5;
            yaw = 0;
        } else if (gamepad1.dpad_down) { // strafe forward
            axial = -.5;
            lateral = 0;
            yaw = 0;
        } else if (gamepad1.dpad_up) { // Strafe backward
            axial = .5;
            lateral = 0;
            yaw = 0;
        } else if(!takeover){ // set control to the sticks
            axial = -gamepad1.left_stick_y / (gamepad1.left_trigger + 1);  // Note: pushing stick forward gives negative value
            lateral = gamepad1.left_stick_x / (gamepad1.left_trigger + 1);
            yaw = gamepad1.right_stick_x / (gamepad1.left_trigger + 1);
        }

        double power_level = 1;
        if (gamepad1.ps) {
            telemetry.addData("Yaw", "Resetting\n");
            follower.setPose(new Pose(poseEstimate.getX(), poseEstimate.getY(), 0));
        }

        //elbow1.setPosition(servo1pose);
        //elbow2.setPosition(servo2pose);

        double leftFrontPower = (axial + lateral + yaw) * power_level;
        double rightFrontPower = (axial - lateral - yaw) * power_level;
        double leftBackPower = (axial - lateral + yaw) * power_level;
        double rightBackPower = (axial + lateral - yaw) * power_level;

        // If the sticks are being used
        if(!gamepad1.dpad_left & !gamepad1.dpad_right & !gamepad1.dpad_up & !gamepad1.dpad_down) {
            double yaw_rad = /*orientation.getYaw(AngleUnit.RADIANS)*/angle + 3.14159 / 2;
            double temp = axial * Math.sin(yaw_rad) + lateral * Math.cos(yaw_rad);
            lateral = -axial * Math.cos(yaw_rad) + lateral * Math.sin(yaw_rad);
            //double temp = axial * Math.cos(yaw_rad) + lateral * Math.sin(yaw_rad);
            //lateral = -axial * Math.sin(yaw_rad) + lateral * Math.cos(yaw_rad);
            axial = temp;
        }
        // Combie the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        if(locked){
            if(tx > 0) {
                if(tx < .3){
                    axial = 0;
                    lateral = 0;
                    yaw = 0;
                }else if(tx < 5){
                    lateral = tx * .2;
                } else {
                    lateral = 1;
                }
            }
            else if (tx < 0){
                if(tx > -.3){
                    axial = 0;
                    lateral = 0;
                    yaw = 0;
                }else if(tx > -5){
                    lateral = tx * .2;
                } else {
                    lateral = -1;
                }
            }
        }
        leftFrontPower = (axial + lateral + yaw) * power_level;
        rightFrontPower = (axial - lateral - yaw) * power_level;
        leftBackPower = (axial - lateral + yaw) * power_level;
        rightBackPower = (axial + lateral - yaw) * power_level;
        // Normalize the values so no wheel power exceeds 00%
        // This ensures that the robot maintains the desired motion.
        double max = Math.max(abs(leftFrontPower), abs(rightFrontPower));
        max = Math.max(max, abs(leftBackPower));
        max = Math.max(max, abs(rightBackPower));
        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }//Arm code Shoulder

        front_left.setPower(leftFrontPower);
        front_right.setPower(rightFrontPower);
        rear_left.setPower(leftBackPower);
        rear_right.setPower(rightBackPower);

    }
    public void longscore(){
        if(scoring == 2 && abs(wristpose - wrist_at) < .02){
            gripspinny.setPower(1);
            wristpose = .5;
            flippose = .55;
            slidestarget = 20;
            scoring = 1;
        }
    }
    public void getwall(){
        if(stick == 2 && abs(wristpose - wrist_at) < .05){
            wristpose = .72;
            stick = 1;
        }
    }
    public void autotwist(){
        if(buttons2.lastbutton == "a" && downish == 1) {
            LLResult result = limelight.getLatestResult();
            double[] pythonOutputs = result.getPythonOutput();
            locked = pythonOutputs[0] >= 1 && (mode == 2 || slidesPose > 500);
            if (locked) {
                mode = 2;
                tx = (pythonOutputs[1])/78 - 4.5; // How far left or right the target is (degrees)
                double ty = (pythonOutputs [2])/78 - 3.6; // How far up or down the target is (degrees)
                double ta = pythonOutputs[5]; // How big the target looks (0%-100% of the image)

                double angle = pythonOutputs[3];
                if(angle < 45 || angle > 135){
                    twistpose = 0;
                }else{
                    twistpose = .28;
                }
                ty += (abs(angle - 90) / 90);
                slidestarget = (int) (slidesPose - (ty * slideticks * 2));
                telemetry.addData("Target X", tx);
                telemetry.addData("Target Y", ty);
                telemetry.addData("Target Area", ta);
                telemetry.addData("Orientation", angle);
                ////////////////////////////////////////////////////////////////

                if(abs(tx) < .7 && abs(ty) < .7){
                    drivetime.reset();
                    takeover = true;
                    gamepad2.rumbleBlips(1);
                    gamepad1.rumbleBlips(1);
                }else{
                    takeover = false;
                }


            } else {
                telemetry.addData("Limelight", "No Targets");
            }

        }else{
            takeover = false;
        }
    }
    public double maxVoltage(double current, double max){
        if(current > max){
            max = current;
        }
        return max;
    }
    public void maxTotalVoltage(){
        if(fl.getCurrent(CurrentUnit.AMPS) + fr.getCurrent(CurrentUnit.AMPS) + rl.getCurrent(CurrentUnit.AMPS) + rr.getCurrent(CurrentUnit.AMPS) > totalmax ){
            totalmax = fl.getCurrent(CurrentUnit.AMPS) + fr.getCurrent(CurrentUnit.AMPS) + rl.getCurrent(CurrentUnit.AMPS) + rr.getCurrent(CurrentUnit.AMPS);
        }
    }
    public void newspit(){
        if(hope == 2) {
            wristpose = .656;
            slidestarget = 0;
            flippose = .12;
        }
        else if(limitwrist1.getState() && hope == 3){
            drivetime.reset();
            hope = 4;
        }else if(hope == 4 && drivetime.time(TimeUnit.MILLISECONDS) > 30){
            hope = 1;
            buttons2.nowbutton = "r2";
        }
    }
    public void lowscore(){
        if(lowpress == 2 && limitwrist1.getState()){
            drivetime.reset();
            lowpress = 3;
        }else if(lowpress == 3 && drivetime.time(TimeUnit.MILLISECONDS) > 80){
            buttons2.nowbutton = "r2";
            gripspinny.setPower(0);
            lowpress = 1;
        }
    }


}
