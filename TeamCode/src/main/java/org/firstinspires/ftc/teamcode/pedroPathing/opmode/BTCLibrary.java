package org.firstinspires.ftc.teamcode.pedroPathing.opmode;

import static java.lang.Math.abs;

import android.security.identity.InvalidReaderSignatureException;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

import java.util.concurrent.TimeUnit;


//@Disabled
public class BTCLibrary extends OpMode {
    ////////////////////////////////////////////
    //The initialization section for all miscelaneous variables
    public Button2 button2 = new Button2();
    public Button1 button1 = new Button1();
    public CameraCalculations.ArmControl armControl = new CameraCalculations.ArmControl();

    /////////////////////////////////////////////////////////

    /////////////////////////////////////////////////////
    //This is for the intake_turret variable
    public double multipick = 1;
    public Servo intake_turret = null;
    public AnalogInput intake_turret_atpos = null;
    public double intake_turret_pos = .51;
    public double turret_range = 327.27272727272727;
    public double turret_degrees_per_tick = 1/turret_range;
    public ElapsedTime turret_counter = new ElapsedTime();
    public double turret_drop = .866;
    public double turret_at = 0;
    public boolean turret_correct = false;

    /////////////////////////////////////////////////////

    /////////////////////////////////////////////////////
    //This is for the intake_twist variables
    public Servo intake_twist = null;
    public AnalogInput intake_twist_atpos = null;
    public ElapsedTime twist_counter = new ElapsedTime();
    public double intake_twist_pos = .5;
    public double twistdegrees = 1/255;
    public double twist_at = 0;
    public boolean twist_correct = false;
    public double twist_offset = 0;

    ///////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////
    //This is for the gripper variables
    public Servo gripper = null;
    public AnalogInput gripper_atpos = null;
    public double gripper_at = 0;
    public double old_gripper_atpos = 0;
    public boolean jammed = false;
    public double rotations_per_second = 0;
    public ElapsedTime speed_counter = new ElapsedTime();
    public boolean drop = false;
    public double open = 1;
    public double nostrain = .78;
    public double closed = .78;
    public boolean jam_pause = false;
    public double gripper_pose = open;
    public boolean gripper_correct = false;
    public ElapsedTime griptime = new ElapsedTime();

    ////////////////////////////////////////////////////////////

    /////////////////////////////////////////////////////////////
    //This is for all slide_servo variables
    public Servo slide_servo = null;
    public Servo slide_servo2 = null;
    public AnalogInput slideatpos = null;
    public double slideservopose = 0;
    public double slideat = 0;
    public boolean slideservocorrect = false;
    public ElapsedTime slide_counter = new ElapsedTime();

    ///////////////////////////////////////////////////

    /////////////////////////////////////////////////
    //This is for the intake's vertical movement variables
    public Servo intake_vertical = null;
    public AnalogInput intake_vertical_atpos = null;
    public ElapsedTime vert_counter = new ElapsedTime();
    public double pick_pose = 0;
    public double ready_pose = 0.07;
    public double drop_pose = .185;
    public double neutral_pose = .2;
    public double intake_vertical_pos = drop_pose;
    public double vert_at = 0;
    public boolean vert_correct = false;
    public double human_player = 1;
    public double modifier = 0;

    /////////////////////////////////////////////////////////

    /////////////////////////////////////////////////////////
    //This is for the chassis variables
    public Follower follower;
    public DcMotor front_left = null;
    public DcMotor rear_left = null;
    public DcMotor front_right = null;
    public DcMotor rear_right = null;

    ////////////////////////////////////////////////////////
    //This is for scoring arm variables
    public DigitalChannel limitslide;
    public double one = 1;
    public PIDController controller;
    public PIDController armcontroller;

    public static double p = 0.004, i = 0, d = 0;

    public static double f = 0.01;

    public static int slidestarget = 0;
    public static double armp = 0.01, armi = 0, armd = 0;

    public static double armf = 0.01;

    public static int armtarget = 0;
    public double armPose = 0;
    double slidesPose = 0;
    public DcMotor slides = null;
    private DcMotor Arm1 = null;
    private DcMotor Arm2 = null;
    private AnalogInput ArmPos = null;
    public boolean slidelimit = false;
    public double tele_start = 0;

    /////////////////////////////////////////////////
    //This is for the scoring gripper variables
    public double wrist_at = 0;
    private Servo wristy = null;
    private Servo twisty = null;
    public spin gripspinny;
    double wristpose = .5;
    double twistpose = .5;
    public double flippose = 0;
    public flippy flip;
    public AnalogInput wristencoder;

    public DigitalChannel limitwrist1;
    public double flipsafe = 1; // This is the variable that turns on the function that makes sure that the twist doesn't jam
    public double stick = 1; // This helps the wrist go up after picking off of the wall

    ///////////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////
    //This is the limelight variables
    public double pic = 1;
    public Limelight3A limelight = null;
    public Servo Light = null;
    public Servo AuditLight = null;
    public boolean pickred = false;
    public boolean pickblue = false;
    public boolean pickyellow = false;
    public boolean manual_picking = false;
    public double forward = 1;
    public boolean has_picked = true;
    public boolean skibidyRizz = false;

    @Override
    public void init(){
        initialize();
    }
    ///////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////
    public void initialize() {

        ////////////////////////////////////////////////
        //This is for servo init
        intake_vertical = hardwareMap.get(Servo.class, "intakevertical");
        intake_twist = hardwareMap.get(Servo.class, "intaketwist");
        intake_turret = hardwareMap.get(Servo.class, "intaketurret");
        gripper = hardwareMap.get(Servo.class, "gripper");
        slide_servo = hardwareMap.get(Servo.class, "slideservo");
        slide_servo2 = hardwareMap.get(Servo.class, "slideservo2");
        intake_vertical.setDirection(Servo.Direction.REVERSE);
        intake_turret.setDirection(Servo.Direction.REVERSE);
        intake_turret_pos = .3;
        intake_vertical_pos = ready_pose;
        slideservopose = 0;
        intake_twist_pos = .5;

        ///////////////////////////////////////////////

        //////////////////////////////////////////////////
        //This is the limelight init
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        Light = hardwareMap.get(Servo.class, "light");
        AuditLight = hardwareMap.get(Servo.class, "auditlight");
        Light.setPosition(.5);
        AuditLight.setPosition(0);
        limelight.pipelineSwitch(5);
        limelight.start();

        ///////////////////////////////////////////////////
        //This is for all analog init
        intake_vertical_atpos = hardwareMap.get(AnalogInput.class, "intakeverticalatpos");
        intake_turret_atpos = hardwareMap.get(AnalogInput.class, "intaketurretatpos");
        intake_twist_atpos = hardwareMap.get(AnalogInput.class, "intaketwistatpos");
        gripper_atpos = hardwareMap.get(AnalogInput.class, "gripperatpos");
        slideatpos = hardwareMap.get(AnalogInput.class, "slideatpos");

        ////////////////////////////////////////////////

        ///////////////////////////////////////////////
        //This is the chassis init
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setPose(new Pose(0,0,0));
        follower.setHeadingOffset(Math.toRadians(0));
        front_left = hardwareMap.get(DcMotor.class, "leftFront");
        front_right = hardwareMap.get(DcMotor.class, "rightFront");
        rear_left = hardwareMap.get(DcMotor.class, "leftRear");
        rear_right = hardwareMap.get(DcMotor.class, "rightRear");
        front_left.setDirection(DcMotor.Direction.REVERSE);
        front_right.setDirection(DcMotor.Direction.FORWARD);
        rear_left.setDirection(DcMotor.Direction.REVERSE);
        rear_right.setDirection(DcMotor.Direction.FORWARD);
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rear_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rear_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ////////////////////////////////////////////////

        //////////////////////////////////////////////////
        //This is for scoring arm init
        controller = new PIDController(p, i, d);
        armcontroller = new PIDController(armp, armi, armd);
        slides = hardwareMap.get(DcMotor.class, "slides"); //0 to -3.5 limit
        Arm1 = hardwareMap.get(DcMotor.class, "Arm1");
        Arm2 = hardwareMap.get(DcMotor.class, "Arm2");
        ArmPos = hardwareMap.get(AnalogInput.class, "ArmPos");
        slides.setDirection(DcMotor.Direction.FORWARD);
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Arm1 .setDirection(DcMotor.Direction.REVERSE);
        Arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armtarget = 732;
        slidestarget = 0;

        //////////////////////////////////////////////////
        //This is for the scoring gripper's init
        limitwrist1 = hardwareMap.get(DigitalChannel.class, "limitwrist1");
        limitslide = hardwareMap.get(DigitalChannel.class, "limitslide");
        wristencoder = hardwareMap.get(AnalogInput.class, "wristencoder");
        gripspinny = new spin();
        flip = new flippy();
        flip.initialize();
        gripspinny.initialize();
        gripspinny.setPower(0);
        wristy = hardwareMap.get(Servo.class, "wrist");
        twisty = hardwareMap.get(Servo.class, "twist");
        wristpose = .43;
        twistpose = .56;
        flippose = .5;
        gamepad2.setLedColor(255,0,255,999999999);
        gamepad1.setLedColor(255,162,0,999999999);

        //////////////////////////////////////////////////
    }

    @Override
    public void loop() {

    }

    /////////////////////////////////////////
    //This is the function that moves the slide servo
    public void slideservomove(){
        double buffer = 0.02;
        double offset = 0.045;
        double max_reach = .7;
        slideat = 1 - (abs(slideatpos.getVoltage() / 3) - offset);
        if (abs(slideservopose / 2 - slideat) < buffer){
           if(abs(gamepad2.right_stick_y) > .1 && slideservopose <= max_reach){
               slideservopose -= gamepad2.right_stick_y * .1;
           }
           if(slide_counter.time(TimeUnit.MILLISECONDS) > 60){
               slideservocorrect = true;
           }else{
               slideservocorrect = false;
           }

        }else{
            slideservocorrect = false;
            slide_counter.reset();
        }
        if(slideservopose > max_reach){
            slideservopose = max_reach;
        }else if(slideservopose < 0){
            slideservopose = 0;
        }
        slide_servo.setPosition(slideservopose / 2);
        slide_servo2.setPosition(slideservopose / 2);
        telemetry.addData("Slide Servo Target", slideservopose / 2);
        telemetry.addData("Slide Servo Position", slideat);
        telemetry.addData("Servo is at position", slideservocorrect);
    }

    ////////////////////////////////////////////////

    ///////////////////////////////////////////////
    //This will control the intake's vertical movement
    public void intake_vertical_move(){
        double offset = -0.05;
        double buffer = .01;
        vert_at = intake_vertical_atpos.getVoltage()/3 + offset - .015;
        intake_vertical_pos = (intake_vertical_pos - modifier) + slideat / .7 * .009;
        modifier = slideat / .7 * .009;
        if (abs(intake_vertical_pos - vert_at) < buffer){
            if(vert_counter.time(TimeUnit.MILLISECONDS) > 60){
                vert_correct = true;
            }else{
                vert_correct = false;
            }

        }else{
            vert_correct = false;
            vert_counter.reset();
        }
        if(intake_vertical_pos == pick_pose && vert_correct){
            gripper_pose = closed;
        }

        if(intake_vertical_pos > 1){
            intake_vertical_pos = 1;
        }else if(intake_vertical_pos < 0){
            intake_vertical_pos = 0;
        }
        intake_vertical.setPosition(intake_vertical_pos + .015);
        telemetry.addData("intake vertical target", intake_vertical_pos);
        telemetry.addData("intake vertical position", vert_at);
        telemetry.addData("intake vertical is at position", vert_correct);
    }

    ///////////////////////////////////////////

    //////////////////////////////////////////////
    //controls the intake's turret
    public void intake_turret_move(){
        double offset = -0.05;
        double buffer = .02;
        double right = -gamepad2.left_stick_y;
        double joystick_angle = Math.toDegrees(Math.atan(abs(right)/abs(gamepad2.left_stick_x)));
        turret_at = intake_turret_atpos.getVoltage()/3 + offset;
        if (abs((intake_turret_pos + .01) - turret_at) < buffer){
            if(turret_counter.time(TimeUnit.MILLISECONDS) > 60){
                turret_correct = true;
            }else{
                turret_correct = false;
            }

        }else{
            turret_correct = false;
            turret_counter.reset();
        }
        if(gamepad2.left_stick_x > 0 && right > 0){
            joystick_angle = 270 - joystick_angle;
        }else if(gamepad2.left_stick_x > 0 && right < 0){
            joystick_angle = 270 + abs(joystick_angle);
        }else if(gamepad2.left_stick_x < 0 && right > 0){
            joystick_angle = 90 + abs(joystick_angle);
        }else{
            joystick_angle = 90 - abs(joystick_angle);
        }

        if(joystick_angle <= turret_range && joystick_angle >= 360 - turret_range && (abs(gamepad2.left_stick_x) + abs(right) >= .95) && right > -.2){
            intake_turret_pos = joystick_angle/turret_range;
        }
        if(intake_turret_pos > 1){
            intake_turret_pos = 1;
        }else if(intake_turret_pos < 0){
            intake_turret_pos = 0;
        }
        intake_turret.setPosition(intake_turret_pos + .01);
        telemetry.addData("Intake Turret Target", intake_turret_pos);
        telemetry.addData("Intake Turret Position", turret_at);
        telemetry.addData("Intake Turret is at Position", turret_correct);
        telemetry.addData("Intake Turret Angle", joystick_angle);

    }

    ///////////////////////////////////////////////

    /////////////////////////////////////////////////
    //This will control the gripper's grabbing and twisting
    public void griptwist(){

        ///////////////////////////////////////
        //gripper section
        double gripoffset = .05;
        double buffer = .01;
        double average_rotations = .06;
        gripper_at = 1 - gripper_atpos.getVoltage()/3 + gripoffset;
        if (abs(gripper_at - gripper_pose) < buffer){
            gripper_correct = true;
        }else{
            gripper_correct = false;
        }
        rotations_per_second = abs((gripper_at - old_gripper_atpos)/speed_counter.time(TimeUnit.SECONDS));
        old_gripper_atpos = 1 - gripper_atpos.getVoltage() / 3 + gripoffset;
        gripper.setPosition(gripper_pose);
        telemetry.addData("Gripper Target Pose", gripper_pose);
        telemetry.addData("Gripper Actual Pose", gripper_at);
        telemetry.addData("average movement", rotations_per_second);
        telemetry.addData("Is the gripper at position", gripper_correct);
        telemetry.addData("Is the gripper jammed", jammed);
        telemetry.addData("Scoring Gripper Limit Switch", !limitwrist1.getState());
        ///////////////////////////////////////

        ///////////////////////////////////////
        //twist section
        double twistoffset = -0.03;
        twist_at = 1 - (intake_twist_atpos.getVoltage()/3 + twistoffset);
        if (abs(intake_twist_pos - twist_at) < buffer){
            if(twist_counter.time(TimeUnit.MILLISECONDS) > 60){
                twist_correct = true;
            }else{
                twist_correct = false;
            }

        }else{
            twist_correct = false;
            twist_counter.reset();
        }
        if(intake_twist_pos > 1){
            intake_twist_pos = 1;
        }else if(intake_twist_pos < 0){
            intake_twist_pos = 0;
        }
        intake_twist.setPosition(intake_twist_pos + .02);
        telemetry.addData("Twist Target", intake_twist_pos);
        telemetry.addData("Twist Position", twist_at);
        telemetry.addData("Twist is at Position", twist_correct);
        ///////////////////////////////////////
    }

    /////////////////////////////////////////////////

    ///////////////////////////////////////////////////

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
            }else if (nowbutton == "r2") {
                armtarget = 732;
                wristpose = .43;
                slidestarget = 0;
                flippose = .025;
                flipsafe = 2;
                if(gripspinny.getPower() == -1){
                    spitting = 2;
                }
                intake_twist_pos = .49;
                intake_turret_pos = .3;
                limelight.pipelineSwitch(5);
                intake_vertical_pos = ready_pose;
                gripper_pose = open;
                slideservopose = 0;
                multipick = 1;
                pic = 1;
                human_player = 1;
                lastbutton = "";
            }else if (nowbutton == "b") {
                intake_turret_pos = .3;
                intake_vertical_pos = ready_pose;
                gripper_pose = open;
            }else if (nowbutton == "l1") {
                pic = 2;
                human_player = 1;
            }else if(nowbutton == "middle"){
                skibidyRizz = true;
                gamepad2.setLedColor(0,255,0,999999999);
            }
            if(lastbutton == ""){
                if (nowbutton == "l3") {
                    armtarget = 2210;
                    wristpose = .34;
                    twistpose = 0;
                    slidestarget = 0;
                    flippose = .583;
                    Light.setPosition(0);
                    lastbutton = "l3";
                    nowbutton = "";
                } else if(nowbutton == "r1"){
                    //Arm moves to pick up stuff from eddy
                    armtarget = 732;
                    wristpose = .43;
                    slidestarget = 0;
                    flippose = .025;
                    pic = 1;
                    if(has_picked){
                        intake_turret_pos = turret_drop + .025;
                        intake_vertical_pos = .22;
                        intake_twist_pos = .77;
                        slideservopose = 0;
                    }
                    Light.setPosition(0);
                    flipsafe = 2;
                    gripspinny.setPower(-1);
                    lastbutton = "r1";
                    nowbutton = "";
                }else if(nowbutton == "a"){
                    slidestarget = 0;
                    intake_twist_pos = .49;
                    intake_turret_pos = .51;
                    pic = 1;
                    gripper_pose = open;
                    intake_vertical_pos = ready_pose;
                    manual_picking = true;
                    lastbutton = "a";
                }
            }else if(lastbutton == "r1"){
                if((nowbutton == "r1" || (!limitwrist1.getState() && wrist_at < .2))){
                    armtarget = 732;
                    slidestarget = 240;
                    wristpose = .62;
                    twistpose = 0;
                    flippose = .56;
                    if(has_picked) {
                        human_player = 2;
                    }
                    Light.setPosition(.5);
                    gripspinny.setPower(-1);
                    lastbutton = "l1";
                    nowbutton = "";
                }
            }
            else if(lastbutton == "l3"){
                if(nowbutton == "l3"){
                    lastbutton = "l32";
                    nowbutton = "";
                }

            }else if(lastbutton == "a"){
                if(nowbutton == "up"){
                    slideservopose = .7;
                }else if(nowbutton == "down"){
                    slideservopose = 0;
                }else if(nowbutton == "right"){
                    intake_twist_pos += .28;
                    if(intake_twist_pos > .77){
                        intake_twist_pos = .77;
                    }
                }else if(nowbutton == "left"){
                    intake_twist_pos -= .28;
                    if(intake_twist_pos < .21){
                        intake_twist_pos = .21;
                    }
                }else if(nowbutton == "a" && pic == 1){
                    pic = 4;
                    lastbutton = "";
                    manual_picking = true;
                }
            }else if(lastbutton == "l1"){
                if (nowbutton == "r1") {
                    //Arm drops block on the hang and goes back in\
                    if(skibidyRizz){
                        wristpose = .45;
                        slidestarget = 300;
                        flippose = .61;
                        gripspinny.setPower(1);
                        lastbutton = "";
                        nowbutton = "";
                    }else {
                        lastbutton = "autodrop";
                        pic = 2;
                        nowbutton = "";
                    }

                }else if(nowbutton == "a"){
                    slidestarget = 0;
                    intake_twist_pos = .49;
                    intake_turret_pos = .51;
                    pic = 1;
                    gripper_pose = open;
                    intake_vertical_pos = ready_pose;
                    manual_picking = true;
                    wristpose = .45;
                    slidestarget = 300;
                    flippose = .61;
                    gripspinny.setPower(1);
                    lastbutton = "a";
                }

            }if(lastbutton == "autodrop"){
                if(follower.getVelocity().getMagnitude() < 4 || nowbutton == "r1") {
                    wristpose = .45;
                    slidestarget = 300;
                    flippose = .61;
                    gripspinny.setPower(1);
                    lastbutton = "";
                    nowbutton = "";
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
            if (nowbutton == "l2") {
                armtarget = 732;
                wristpose = .43;
                slidestarget = 0;
                flippose = .025;
                flipsafe = 2;
                if(gripspinny.getPower() == -1){
                    spitting = 2;
                }
                intake_twist_pos = .49;
                intake_turret_pos = .3;
                limelight.pipelineSwitch(5);
                intake_vertical_pos = ready_pose;
                gripper_pose = open;
                slideservopose = 0;
                multipick = 1;
                pic = 1;
                human_player = 1;
                lastbutton = "";
            }else if(lastbutton == "") {
                if(nowbutton == "r2"){
                    far_human_player = 2;
                }
                else if (nowbutton == "a") {
                    multipick = 2;
                    limelight.pipelineSwitch(4);
                    pic = 2.5;
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

    ///////////////////////////////////////////////////////

    //////////////////////////////////////////////////////
    //This is the code for the scoring arm
    public void arm(){
        double slideratio = 2;
        double slideticks = 103.8 * slideratio / 4.75;
        double armticks = 8192 / 360;
        double ticks = .002866;
        double toplimit = 7 * slideticks * 2;
        wrist_at = abs(1 - wristencoder.getVoltage() / 3) + .03;
        controller.setPID(p, i, d);
        slidesPose = -slides.getCurrentPosition() * 2;
        armd = (-slides.getCurrentPosition())/slideticks * .03 / 19.6;
        armf = .001 + (-slides.getCurrentPosition())/slideticks * .2 / 19.6;
        double pid = controller.calculate(slidesPose, slidestarget);
        double ff = Math.cos(Math.toRadians(slidestarget)) * f;
        double power = pid + ff;
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
            slides.setPower(-power);
        }
        armcontroller.setPID(armp, armi, armd);
        armPose = (1 - ArmPos.getVoltage() - .2) / ticks * armticks;
        double armpid = controller.calculate(armPose, armtarget);
        double armff = Math.cos(Math.toRadians(armtarget)) * armf;
        double armpower = armpid + armff;
        if(button2.lastbutton == "l32"){
            if(armPose < 30){
                armtarget = 0;
                button2.lastbutton = "";
            }else{
                armpower = -1;
                Arm1.setPower(-armpower);
                Arm2.setPower(-armpower);
            }

        }
        Arm1.setPower(-armpower);
        Arm2.setPower(-armpower);
        flip.setPosition(flippose);
        wristy.setPosition(wristpose - .04);
        twisty.setPosition(twistpose + .028);

    }

    //////////////////////////////////////////////////////

    //////////////////////////////////////////////////////
    //This is to keep the twist from jamming when picking up from the human player
    public void safeflip(){
        if(flipsafe == 2 && flip.getPosition() < .45){
            twistpose = .56;
            flipsafe = 1;
        }
    }

    //////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////
    //This helps the wrist go up when the specimen is picked off the wall
    public void getwall(){
        if(stick == 2 && abs(wristpose - wrist_at) < .05){
            wristpose = .72;
            stick = 1;
        }
    }

    ///////////////////////////////////////////////////////

    //////////////////////////////////////////////////////
    //controls the chassis movement
    public void drive(){
        follower.update();
        if(pic < 2.5 || pic >= 4.5) {
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
            } else { // set control to the sticks
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
            if (!gamepad1.dpad_left & !gamepad1.dpad_right & !gamepad1.dpad_up & !gamepad1.dpad_down) {
                double yaw_rad = /*orientation.getYaw(AngleUnit.RADIANS)*/angle + 3.14159 / 2;
                double temp = axial * Math.sin(yaw_rad) + lateral * Math.cos(yaw_rad);
                lateral = -axial * Math.cos(yaw_rad) + lateral * Math.sin(yaw_rad);
                //double temp = axial * Math.cos(yaw_rad) + lateral * Math.sin(yaw_rad);
                //lateral = -axial * Math.sin(yaw_rad) + lateral * Math.cos(yaw_rad);
                axial = temp;
            }
            // Combie the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.

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

    }

    ///////////////////////////////////////////////////

    ///////////////////////////////////////////////////
    //This is the flip-a-grip class
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
            return abs(1 - flipencoder.getVoltage() / 3) + .03;
        }
    }

    /////////////////////////////////////////////////////

    ////////////////////////////////////////////////////
    //This class controls the scoring intake wheels
    public class spin{
        public CRServo spinny1;
        public CRServo spinny2;
        public void initialize(){
            spinny1 = hardwareMap.get(CRServo.class, "spinny1");
            spinny2 = hardwareMap.get(CRServo.class, "spinny2");
            spinny2.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        public void setPower(double power){
            spinny1.setPower(power);
            spinny2.setPower(power);
        }
        public double getPower(){
            return spinny1.getPower();
        }
    }
    public void takepicture(){
        LLResult result = limelight.getLatestResult();
        double[] pythonOutputs = result.getPythonOutput();
        double[] positions = armControl.main(pythonOutputs[0], pythonOutputs[1], pythonOutputs[2], armPose, -slides.getCurrentPosition());
        if(pic == 1 && forward == 1){
            if(positions[0] < 1 && positions[1] < 1 && positions[2] < 1 && positions[0] > 0 && positions[1] > 0 && positions[2] > 0){
                gamepad1.rumbleBlips(1);
                if(follower.getVelocity().getMagnitude() < 5) {
                    gamepad2.rumbleBlips(1);

                }

            }
        }
        if(pic > 1) {
            AuditLight.setPosition(.5);
            if(pic == 2 && turret_correct && positions[0] < 1 && positions[1] < 1 && positions[2] < 1 && pythonOutputs[1] > 0 && (follower.getVelocity().getMagnitude() < 5 || forward != 1)) {
                if(multipick == 1){
                    intake_vertical_pos = ready_pose;
                    gripper_pose = open;
                }
                follower.holdPoint(new Pose(follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading()));
                pic = 2.5;
            } else if (vert_correct && pic == 2.5) {
                intake_vertical_pos = ready_pose;
                gripper_pose = open;
                if(positions[0] >= 1 || positions[1] >=1 || positions[2] >= 1){
                    pic = 1;
                    if(multipick > 1){
                        multipick = 1;
                        limelight.pipelineSwitch(5);
                    }
                }else {
                    intake_turret_pos = positions[0];
                    slideservopose = positions[1];
                    if(slideservopose < 0){
                        slideservopose = 0;
                    }
                    intake_twist_pos = positions[2];
                    telemetry.addData("TurretPose", pythonOutputs[0]);
                    telemetry.addData("SlidePose", pythonOutputs[1]);
                    telemetry.addData("TwistPose", pythonOutputs[2]);
                    telemetry.update();
                    if(multipick > 1){
                        picks += 1;
                    }
                    pic = 4;
                }
            }else if(pic == 4 && ((slideservocorrect && turret_correct && twist_correct) || gamepad1.dpad_up || manual_picking || forward == 11)){
                intake_vertical_pos = pick_pose;
                pic = 4.25;

            }else if(pic == 4.25){
                if(vert_at <= pick_pose + .025){
                    gripper_pose = closed;
                    pic = 4.4;
                }
            }else if(pic == 4.4 && (gripper_correct || gripper_at < .77)){
                intake_vertical_pos = drop_pose;
                has_picked = true;
                pic = 4.5;
            }else if(vert_at > .05 && pic == 4.5){
                follower.breakFollowing();
                front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rear_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rear_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                intake_turret_pos = .51;
                intake_vertical_pos = drop_pose;
                intake_twist_pos = .77;
                slideservopose = 0;
                manual_picking = false;

                if(multipick > 1){
                    human_player = 2;
                }
                pic = 1;
            }


        }else{
            AuditLight.setPosition(0);
        }
    }
    public void drop_at_human_player(){
        if(human_player == 2){
            intake_turret_pos = turret_drop + .025;
            intake_vertical_pos = .22;
            has_picked = false;
            if(pic == 0){
                intake_vertical_pos += .03;
                intake_twist_pos = .49;
            }else {
                intake_twist_pos = .77;
            }
            slideservopose = 0;
            human_player = 3;
            if(multipick > 1){
                if(picks == 3 && forward > 1){
                    multipick = 1;
                    picks = 0;
                    human_player = 1;
                }else {
                    intake_vertical_pos = .23;
                    intake_turret_pos = .13;
                }
            }

        }else if(human_player == 3 && ((turret_correct && slideservocorrect && vert_correct) || !limitwrist1.getState())){
            gripper_pose = open;
            human_player = 4;
        }else if(human_player == 4 && gripper_at > open - .15){
            if(multipick == 1) {
                if(forward == 1) {
                    intake_turret_pos = .5;
                }else{
                    intake_turret_pos = .3;
                }
                slideservopose = 0;
                intake_vertical_pos = .2;
                human_player = 5;
            }else{
                if(picks == 3 && forward > 1){
                    multipick = 1;
                    picks = 0;
                }else {
                    pic = 2.5;
                    human_player = 1;
                }
            }
        }else if(human_player == 5 && turret_correct){
            intake_twist_pos = .77;
            intake_vertical_pos = ready_pose;
            human_player = 1;
        }
    }
    public double far_human_player = 1;
    public void far_human_player(){
        if(far_human_player == 2){
            intake_turret_pos = .51;
            intake_vertical_pos = ready_pose;
            intake_twist_pos = .49;
            slideservopose = .6;
            far_human_player = 3;
        }else if(far_human_player == 3 && turret_correct && slideservocorrect && vert_correct){
            gripper_pose = open;
            far_human_player = 4;
        }else if(far_human_player == 4 && gripper_correct){
            intake_turret_pos = .25;
            slideservopose = 0;
            intake_vertical_pos = .32;
            intake_twist_pos = .5;
            far_human_player = 5;
        }else if(far_human_player == 5 && turret_correct){
            intake_vertical_pos = .25;
            far_human_player = 1;
        }
    }
    public double vert_tranfer = .1;
    public double twist_transfer = .1;
    public double shoulder_transfer = 0;
    public double slide_transfer = 0;
    public double intakeslide_transfer = .5;
    public double gripper_transfer = .5;
    public double turret_transfer = .5;
    public double wrist_transfer = .5;
    public double scoretwist_transfer = .5;
    public double flip_transfer = .5;
    public double transfer = 1;
    public void transfer(){
        if(transfer >= 2) {
            if (transfer == 2) {
                slidestarget = (int) slide_transfer;
                armtarget = (int) shoulder_transfer;
                wristpose = wrist_transfer;
                twistpose = scoretwist_transfer;
                flippose = flip_transfer;
                transfer = 3;
            } else if (transfer == 3 && slidesPose < 10 && armPose < 5 && abs(flip.getPosition() - flippose) < .04 && abs(wrist_at - wristpose) < .04) {

            }
        }
    }


    public double picks = 0;
    public void feedgreen(){
        if(multipick > 1){
            if(picks == 2){
                limelight.pipelineSwitch(5);
            }
        }
    }
    public double spitting = 1;
    public ElapsedTime spit_timer = new ElapsedTime();
    public void wait_to_spit(){
        if(spitting == 2 && wrist_at < .3){
            gripspinny.setPower(1);
            spitting = 3;
            spit_timer.reset();
        }if(spitting == 3 && spit_timer.time(TimeUnit.MILLISECONDS) > 350){
            gripspinny.setPower(-1);
            if(button2.lastbutton == "l1") {
                button2.lastbutton = "r1";
            }
            spitting = 1;
        }
    }

}
