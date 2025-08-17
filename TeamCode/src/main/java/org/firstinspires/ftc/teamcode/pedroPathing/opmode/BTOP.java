package org.firstinspires.ftc.teamcode.pedroPathing.opmode;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name="BTCop", group="ABC Opmode")
public class BTOP extends BTCLibrary {

    @Override
    public void loop(){
        button1.button();
        button2.button();
        first();
        if(button1.button != "" || button2.button != "" || abs (gamepad1.left_stick_y) > .1){
            tele_start = 1;
        }
        if(tele_start == 1) {
            slideservomove();
            intake_turret_move();
            griptwist();
            intake_vertical_move();
            arm();
            safeflip();
            getwall();
            wait_to_spit();
            drive();

            takepicture();
            drop_at_human_player();
            far_human_player();
        }
        telemetry.update();
    }
}
