package pedroPathing.opmode;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Config
@TeleOp(name="NewArmTest", group="ABC Opmode")
//@Disabled
public class NewArmTest extends OpMode {
    public Servo slide_servo = null;
    public double slidepose = 1;
    public double slideat = 0;
    public AnalogInput slideatpos = null;
    @Override
    public void init() {
        slideatpos = hardwareMap.get(AnalogInput.class, "slideatpos");
        slide_servo = hardwareMap.get(Servo.class, "slideservo");
    }

    @Override
    public void loop() {
        slideat = abs(1 - slideatpos.getVoltage() / 3.3);
        if(gamepad1.x){
            slidepose = .8;
        }
        else if(gamepad1.y){
            slidepose = .6;
        }
        else if(gamepad1.a){
            slidepose = .5;
        }
        else if(gamepad1.b){
            slidepose = .2;
        }
        else if(gamepad1.dpad_up){
            slidepose = 0;
        }else if(gamepad1.dpad_down){
            slidepose = 1;
        }
        if(slidepose > 1){
            slidepose = 1;
        }else if(slidepose < 0){
            slidepose = 0;
        }
        slide_servo.setPosition(slidepose);
        telemetry.addData("Slide Pose = ", slideat);
        telemetry.update();
    }



}



