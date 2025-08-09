package org.firstinspires.ftc.teamcode.pedroPathing.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;


@Config
@TeleOp(name="MovingPickTest", group="ABC Opmode")
//@Disabled
public class MovingPickTest extends BTCLibrary {
    public Servo test_servo = null;

    public boolean uppress = false;
    public boolean downpress = false;
    public double start_x = 0;
    public double start_y = 0;
    public double y_offset = 0;
    public double x_offset = 0;
    public double H_C = 13;
    private static final double MID_POSITION = 0.485;
    private static final double TICKS_IN_ROTATION = 1 / 327.2727272;
    private static final double MAX_ROTATION = 327.27272727272;
    public static double GRIPPER_STICK_OUT = 11.35;
    private static final double SLIDE_INCH_PER_TICK = 10.25 / MID_POSITION;
    @Override
    public void init() {
        initialize();
    }

    @Override
    public void loop() {
        slideservomove();
        intake_turret_move();
        griptwist();
        intake_vertical_move();
        follower.update();
        if(gamepad1.dpad_up){
            uppress = true;
        }else if(!gamepad1.dpad_up && uppress){
            slideservopose = .3;
            intake_turret_pos = .5;
            intake_vertical_pos = .08;
            intake_twist_pos = .5;
            gripper_pose = open;
            start_x = follower.getPose().getX();
            start_y = follower.getPose().getY();
            x_offset = 0;
            y_offset = 0;
            uppress = false;
        }
        if(gamepad1.dpad_down){
            downpress = true;
        }else if(!gamepad1.dpad_down && downpress) {
            downpress = false;
        }


        telemetry.update();
    }
    public double[] armpick(double x, double y, double robotangle, double old_turret, double old_slides, double old_twist) {
        double turret = 2;
        double slides = 2;


        double angle = 1;
        double twist;

        double slideMove = 0;
        angle = old_turret - (x / GRIPPER_STICK_OUT * 90 * TICKS_IN_ROTATION + MID_POSITION);
        twist = old_twist - (angle - MID_POSITION);

        if (twist < 0.21) {
            twist = 0.77 - (0.21 - twist);
        } else if (twist > 0.77) {
            twist = 0.21 + (twist - 0.77);
        }
        if(x > 0){
            y -= x/8 * 2.5;
        }
        slideMove = y - ((GRIPPER_STICK_OUT) - Math.abs((angle - MID_POSITION) / TICKS_IN_ROTATION) / 90 * 8);
        slideMove -= slideMove /  23;

        slides = old_slides + (slideMove / SLIDE_INCH_PER_TICK);
        turret = angle;



        return new double[]{turret, slides, twist};
    }



}



