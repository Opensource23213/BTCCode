package pedroPathing.constants;

import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Localizers;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class AltFConstants {
    static {
        FollowerConstants.localizers = Localizers.PINPOINT;

        FollowerConstants.leftFrontMotorName = "front_left";
        FollowerConstants.leftRearMotorName = "rear_left";
        FollowerConstants.rightFrontMotorName = "front_right";
        FollowerConstants.rightRearMotorName = "rear_right";

        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;

        FollowerConstants.mass = 31;

        FollowerConstants.xMovement = 79.6;
        FollowerConstants.yMovement = 58.7;
        FollowerConstants.useVoltageCompensationInAuto = true;
        FollowerConstants.forwardZeroPowerAcceleration = -39.6;
        FollowerConstants.lateralZeroPowerAcceleration = -78.8;

        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.8,0.0001,0.05,0);
        FollowerConstants.useSecondaryTranslationalPID = true;
        FollowerConstants.secondaryTranslationalPIDFFeedForward = .01;
        FollowerConstants.translationalPIDFSwitch = 2;
        FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(0.1,0,0.05,0); // Not being used, @see useSecondaryTranslationalPID
        FollowerConstants.headingPIDFCoefficients.setCoefficients(2,0,0.1,0);
        FollowerConstants.useSecondaryHeadingPID = false;
        FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(2,0,0.1,0); // Not being used, @see useSecondaryHeadingPID

        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.008,0,2,0.6,0);
        FollowerConstants.useSecondaryDrivePID = true;
        FollowerConstants.drivePIDFSwitch = 50;
        FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(0.003,0,2,0.6,0); // Not being used, @see useSecondaryDrivePID

        FollowerConstants.zeroPowerAccelerationMultiplier = 5.75;
        FollowerConstants.centripetalScaling = 0.0002;
        FollowerConstants.automaticHoldEnd = true;

        FollowerConstants.pathEndTimeoutConstraint = 500;
        FollowerConstants.pathEndTValueConstraint = 0.995;
        FollowerConstants.pathEndVelocityConstraint = 0.1;
        FollowerConstants.pathEndTranslationalConstraint = 0.1;
        FollowerConstants.pathEndHeadingConstraint = 0.007;
    }
}
