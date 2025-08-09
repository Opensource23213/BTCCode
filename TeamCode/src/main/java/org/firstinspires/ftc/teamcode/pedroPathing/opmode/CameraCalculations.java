package org.firstinspires.ftc.teamcode.pedroPathing.opmode;

public class CameraCalculations {

    public static class ArmControl {

        public double H_C = 13;
        private static final double MID_POSITION = 0.485;
        private static final double TICKS_IN_ROTATION = 1 / 327.2727272;
        private static final double MAX_ROTATION = 327.27272727272;
        public static double GRIPPER_STICK_OUT = 11.35;
        private static final double SLIDE_INCH_PER_TICK = 10.25 / MID_POSITION;

        public double[] armpick(double x, double y, double blockAngle) {
            double turret = 2;
            double slides = 2;
            double xInch = x;
            boolean moveOn = false;

            double circumference = GRIPPER_STICK_OUT * Math.PI * 2;
            double tempAngle = blockAngle;

            if (tempAngle > 90) {
                tempAngle = 180 - tempAngle;
            }

            blockAngle += ((x / 7) * (tempAngle / 90) * 12);
            blockAngle += 90;
            if (blockAngle > 180) {
                blockAngle -= 180;
            } else if (blockAngle < 0) {
                blockAngle = 0 + blockAngle;
            }

            double angle = 1;
            double twist;

            if (x > GRIPPER_STICK_OUT) {
                moveOn = true;
            } else {
                angle = x / GRIPPER_STICK_OUT * 90 + 90;
                double cameraDistanceFromBase = 2.75;
                y += cameraDistanceFromBase;

                double slideMove = 0;

                angle = (MAX_ROTATION / 2) - (90 - angle);
                twist = (blockAngle - 90) * TICKS_IN_ROTATION + 0.49;
                double originalAngle = angle;
                angle = x / GRIPPER_STICK_OUT * 90 * TICKS_IN_ROTATION + MID_POSITION;
                twist -= angle - MID_POSITION;

                if (twist < 0.21) {
                    twist = 0.77 - (0.21 - twist);
                } else if (twist > 0.77) {
                    twist = 0.21 + (twist - 0.77);
                }
                if(x > 0){
                    y -= x/8 * 2.5;
                }
                if(x > 0 && y < 18){
                    y += 1;
                }
                if(x < 0 && y < 20){
                    y += .5
                    ;
                }
                if(y >= 18){
                    slideMove -= .5;
                }
                slideMove = y - ((GRIPPER_STICK_OUT) - Math.abs((angle - MID_POSITION) / TICKS_IN_ROTATION) / 90 * 8);
                slideMove -= slideMove /  12;

                if (!moveOn) {
                    slides = slideMove / SLIDE_INCH_PER_TICK;
                    turret = angle;
                } else {
                    turret = 5;
                    slides = 5;
                    twist = 5;
                }

                return new double[]{turret, slides, twist};
            }

            // If moveOn is true from the start
            turret = 5;
            slides = 5;
            twist = 5;
            return new double[]{turret, slides, twist};
        }

        public double getDistanceFromVerticalPixel(double xPix, double h_c, double vFoV, double theta, int R) {
            double angleOffset = ((xPix - R / 2.0) / R) * vFoV;
            double totalAngleDeg = theta + angleOffset;
            double totalAngleRad = Math.toRadians(totalAngleDeg);
            return h_c / Math.tan(totalAngleRad);
        }

        public double getHorizontalDisplacement(double xPixHorizontal, double y, double hFoV, int W) {
            double angleOffsetDeg = ((xPixHorizontal - W / 2.0) / W) * hFoV;
            double angleOffsetRad = Math.toRadians(angleOffsetDeg);
            return (y * 0.77 + 9) * Math.tan(angleOffsetRad);
        }

        public double[] getGroundCoordinates(double xPixVertical, double xPixHorizontal, double h_c, double vFoV, double theta, int R, double hFoV, int W) {
            double y = getDistanceFromVerticalPixel(xPixVertical, h_c, vFoV, theta, R);
            double X = getHorizontalDisplacement(xPixHorizontal, y, hFoV, W);
            return new double[]{X, y};
        }

        public double[] main(double xpix, double ypix, double angle, double armangle, double slides) {
            double[] arm = armpick(xpix * -1, ypix, angle);
            return arm;
        }
    }
}
