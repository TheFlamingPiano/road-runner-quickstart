package org.firstinspires.ftc.teamcode;

public class InverseKinematics {
    double length1; // length of arm 1
    double length2; // length of arm 2

    double [] retval2;
    double [] retval3;
    double [] testPoint;

    public InverseKinematics(double arm1_length, double arm2_length) {
        length1 = arm1_length;
        length2 = arm2_length;

        retval2 = new double[2];
        retval3 = new double[3];
        testPoint = new double[2];
    }

    public double[] getAngles(double x, double y) {
        double tmp = (x*x + y*y - length1 * length1 - length2 * length2) / (2 * length1 * length2);
        double theta2 = Math.atan2(-Math.sqrt(1 - tmp * tmp), tmp);
        double k1 = length1 + length2 * Math.cos(theta2);
        double k2 = length2 * Math.sin(theta2);
        double theta1 = Math.atan2(y, x) - Math.atan2(k2, k1);
        retval2[0] = Math.toDegrees(theta1);
        retval2[1] = Math.toDegrees(theta2);

        return retval2;
    }

    public double[] getPoint(double angle1, double angle2) {
        testPoint[0] = length1 * Math.cos(Math.toRadians(angle1)) + length2 * Math.cos(Math.toRadians(angle1 + angle2));
        testPoint[1] = length1 * Math.sin(Math.toRadians(angle1)) + length2 * Math.sin(Math.toRadians(angle1 + angle2));
        return testPoint;
    }

    public double[] getAngles(double x, double y, double z) {
        double [] angles = getAngles(Math.hypot(x, y), z);
        retval3[0] = Math.toDegrees(Math.atan2(y, x));
        retval3[1] = angles[0];
        retval3[2] = angles[1];

        return retval3;
    }
}