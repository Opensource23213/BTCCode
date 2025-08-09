package org.firstinspires.ftc.teamcode.pedroPathing.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Config
@TeleOp(name="ColorSensorTest", group="ABC Opmode")
//@Disabled
public class ColorSensorTest extends OpMode {
    public Servo gripper = null;
    public Servo vert = null;
    public static double testpose = 1; // Set this to where you want the servo to start
    public static double vert_pos = .92;
    public double testat = 0;
    public double open = 1;
    public double closed = .25;
    public RevColorSensorV3 color = null;
    public boolean uppress = false;
    public boolean downpress = false;
    public boolean sense = false;
    public double blue = 1000;
    public double red = 0;
    public double yellow = 0;
    public double current_blue = 0;
    public double current_red = 0;
    public double current_green = 0;
    public double rgb = 0;
    public double distance = 0;
    public double current_alpha = 0;
    public static boolean pickred = false;
    public static boolean pickblue = false;
    public static boolean pickyellow = false;
    @Override
    public void init() {
        gripper = hardwareMap.get(Servo.class, "gripper");
        vert = hardwareMap.get(Servo.class, "intakevertical");
        color = hardwareMap.get(RevColorSensorV3.class, "color");
    }

    @Override
    public void loop() {
        vert.setPosition(vert_pos);
        current_blue = color.blue();
        current_green = color.green();
        current_red = color.red();
        current_alpha = color.alpha();
        distance = color.getDistance(DistanceUnit.CM);
        color_sense();
        if(gamepad1.a){
            uppress = true;
        }else if(!gamepad1.a && uppress){
            testpose = open;
            uppress = false;
        }
        rgb = current_green - current_red + current_blue;
        gripper.setPosition(testpose);
        telemetry.addData("blue", current_blue);
        telemetry.addData("red", current_red);
        telemetry.addData("green", current_green);
        telemetry.addData("alpha", current_alpha);
        telemetry.addData("green - red + blue", rgb);
        telemetry.addData("distance", distance);
        telemetry.update();
    }
    public void color_sense(){
        if(distance < 2) {
            if (current_red < 500 && rgb > 500 && pickblue && current_green < 1000 && current_blue > 1000) {
                testpose = .75;
                pickblue = false;
            } else if (current_red > 1000 && rgb < 0 && pickred && current_green < 800 && current_blue < 500) {
                testpose = .75;
                pickred = false;
            } else if(pickyellow){
                testpose = .75;
                pickyellow = false;
            }
        }
    }



}



