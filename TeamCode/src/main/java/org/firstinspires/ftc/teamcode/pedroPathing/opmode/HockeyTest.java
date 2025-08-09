package org.firstinspires.ftc.teamcode.pedroPathing.opmode;

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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

/**
 * This is the StraightBackAndForth autonomous OpMode. It runs the robot in a specified distance
 * straight forward. On reaching the end of the forward Path, the robot runs the backward Path the
 * same distance back to the start. Rinse and repeat! This is good for testing a variety of Vectors,
 * like the drive Vector, the translational Vector, and the heading Vector. Remember to test your
 * tunings on CurvedBackAndForth as well, since tunings that work well for straight lines might
 * have issues going in curves.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/12/2024
 */
@Config
@Autonomous (name = "Hockey", group = "Auto")
public class HockeyTest extends BTCLibrary {
    private Telemetry telemetryA;

    public static double DISTANCE = 60;

    private double forward = 1;


    private Path forwards;
    private Path turn2;
    private Path turn1;
    private Path turn3;
    private Path turn4;

    /**
     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
     * initializes the FTC Dashboard telemetry.
     */
    @Override
    public void init() {
        initialize();
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        controller = new PIDController(p, i, d);
        follower.setStartingPose(new Pose(24,0));
        forwards = new Path(new BezierCurve(new Point(24,0, Point.CARTESIAN), new Point(24,-25, Point.CARTESIAN), new Point(58,-25, Point.CARTESIAN), new Point(58,-27, Point.CARTESIAN),new Point(15,-40, Point.CARTESIAN)));
        forwards.setConstantHeadingInterpolation(0);
        turn1 = new Path(new BezierCurve(new Point(14,-43.5, Point.CARTESIAN), new Point(3,-43.5, Point.CARTESIAN)));
        turn1.setConstantHeadingInterpolation(0);
        turn2 = new Path(new BezierCurve(new Point(3,-43.5, Point.CARTESIAN), new Point(8,-38, Point.CARTESIAN)));
        turn2.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(75));
        turn3 = new Path(new BezierCurve(new Point(8,-38, Point.CARTESIAN), new Point(12.3,0, Point.CARTESIAN), new Point(21.3,10, Point.CARTESIAN), new Point(30,12, Point.CARTESIAN), new Point(36,12, Point.CARTESIAN)));
        armtarget = 732;
        wristpose = .43;
        slidestarget = 0;
        flippose = .025;
        intake_turret_pos = .7;
        intake_vertical_pos = ready_pose;
        intake_twist_pos = .5;
        gripper_pose = open;
        follower.followPath(forwards);

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("This will run the robot in a straight line going " + DISTANCE
                            + " inches forward. The robot will go forward and backward continuously"
                            + " along the path. Make sure you have enough room.");
        telemetryA.update();
    }

    @Override
    public void init_loop(){
        arm();
        slideservomove();
        intake_turret_move();
        griptwist();
        intake_vertical_move();
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the FTC Dashboard.
     */
    @Override
    public void loop() {
        arm();
        slideservomove();
        intake_turret_move();
        griptwist();
        intake_vertical_move();
        takepicture();
        drop_at_human_player();
        follower.update();
        if(one <= 3) {
            if (follower.getPose().getY() < -30 && one == 2) {
                slideservopose = .55;
                intake_turret_pos = .51;
                intake_vertical_pos = .025;
            }

            if (follower.getPose().getY() < -20 && follower.getPose().getX() < 30 && one == 1) {
                follower.breakFollowing();
                one = 2;
                follower.setMaxPower(.5);
                follower.holdPoint(new Pose(16, -43.5));
            }
            if (follower.getPose().getY() < -41 && one == 2) {
                gripper_pose = closed;
                gripspinny.setPower(-1);
                one = 3;
            }
            if (gripper_correct && one == 3) {
                human_player = 2;
                follower.followPath(turn1);
                one = 4;
            }
        }else if(one == 4){
            if(follower.atParametricEnd()){
                if(forward == 1){
                    follower.followPath(turn2);
                    follower.setMaxPower(1);
                    flippose = .2;
                    forward = 2;
                }else if(forward == 2){
                    follower.followPath(turn3);
                    armtarget = 732;
                    wristpose = .75;
                    twistpose = 0;
                    flippose = .65;
                    slidestarget = 648;
                    forward = 3;
                }
            }
        }



        telemetryA.addData("going forward", forward);
        follower.telemetryDebug(telemetryA);
    }
}
