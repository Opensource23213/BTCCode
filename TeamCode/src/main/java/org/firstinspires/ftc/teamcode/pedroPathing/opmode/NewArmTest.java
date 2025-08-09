package org.firstinspires.ftc.teamcode.pedroPathing.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;


@Config
@TeleOp(name="NewArmTest", group="ABC Opmode")
//@Disabled
public class NewArmTest extends OpMode {
    public Servo test_servo = null;
    public static double testpose = 0; // Set this to where you want the servo to start
    public double testat = 0;
    public AnalogInput testatpos = null;
    public boolean uppress = false;
    public boolean downpress = false;
    @Override
    public void init() {
        test_servo = hardwareMap.get(Servo.class, "intakevertical");
        test_servo.setDirection(Servo.Direction.REVERSE);
    }

    @Override
    public void loop() {
        if(gamepad1.dpad_up){
            uppress = true;
        }else if(!gamepad1.dpad_up && uppress){
            testpose += .005;
            uppress = false;
        }
        if(gamepad1.dpad_down){
            downpress = true;
        }else if(!gamepad1.dpad_down && downpress) {
            testpose -= .005;
            downpress = false;
        }
        if(gamepad1.a){
            testpose = .6;
        }
        if(testpose > 1){
            testpose = 1;
        }else if(testpose < 0){
            testpose = 0;
        }
        test_servo.setPosition(testpose);
        telemetry.addData("Slide Actual Pose = ", testat);
        telemetry.addData("Slide Pose = ", testpose);
        telemetry.update();
    }



}



