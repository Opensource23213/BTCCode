package org.firstinspires.ftc.teamcode.pedroPathing.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@Config
@TeleOp(name="CameraTest", group="ABC Opmode")
//@Disabled
public class CameraTest extends BTCLibrary {

    public boolean apress = false;
    public boolean bpress = false;
    public boolean xpress = false;
    public boolean ypress = false;
    public boolean uppress = false;
    public boolean downpress = false;


    @Override
    public void loop() {
        slidestarget = 648;
        arm();
        if(gamepad1.a){
            apress = true;
        }else if(!gamepad1.a && apress){
            pic = 2;
            apress = false;
        }
        if(gamepad1.b){
            bpress = true;
        }else if(!gamepad1.b && bpress){
            intake_turret.setPosition(.7);
            slide_servo.setPosition(0);
            intake_twist.setPosition(.5);
            bpress = false;
        }
        if(gamepad1.x){
            xpress = true;
        }else if(!gamepad1.x && xpress){
            intake_vertical.setPosition(pick_pose);
            xpress = false;
        }
        if(gamepad1.y){
            ypress = true;
        }else if(!gamepad1.y && ypress){
            intake_vertical.setPosition(drop_pose);
            ypress = false;
        }
        if(gamepad1.dpad_up){
            uppress = true;
        }else if(!gamepad1.dpad_up && uppress){
            gripper.setPosition(nostrain);
            uppress = false;
        }
        if(gamepad1.dpad_down){
            downpress = true;
        }else if(!gamepad1.dpad_down && downpress){
            gripper.setPosition(closed);
            downpress = false;
        }
        takepicture();
    }




}



