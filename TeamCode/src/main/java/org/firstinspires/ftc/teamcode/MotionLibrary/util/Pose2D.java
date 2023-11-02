package org.firstinspires.ftc.teamcode.MotionLibrary.util;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

/**
 *  A class that contains an X, Y and heading value.
 *  These can be used as coordinates with a heading value to help with localization.
 */
public class Pose2D {

    public double X = 0;
    public double Y = 0;
    public double heading = 0;

    /**
     * Creates a new Pose2d with the specified X, Y, and heading values
     * @param x
     * @param y
     * @param Heading
     */
    public Pose2D(double x, double y, double Heading) {
        X = x;
        Y = y;
        heading = Heading;
    }

    /**
     * Creates a new Pose2d with the specified X and Y values and a heading of 0
     * @param x
     * @param y
     */
    public Pose2D(double x, double y) {
        X = x;
        Y = y;
    }

    /**
     * Creates a new Pose2d with the X and Y values equal to the parameter and a heading of 0
     * @param XAndY
     */
    public Pose2D(double XAndY) {
        X = XAndY;
        Y = XAndY;
    }

    /**
     * Creates a new Pose2d with the X and Y values equal to the Vector2d X and Y values and a heading of 0
     * @param vector2d
     */
    public Pose2D(Vector2D vector2d) {
        X = vector2d.X;
        Y = vector2d.Y;
    }

    /**
     * Creates a new Pose2d with the X and Y values equal to the Vector2d X and Y values and a heading of the parameter
     * @param vector2d
     * @param heading
     */
    public Pose2D(Vector2D vector2d, double heading) {
        X = vector2d.X;
        Y = vector2d.Y;
        this.heading = heading;
    }

    /**
     * Creates a new Pose2d with the X, Y and heading at 0
     */
    public Pose2D() {}

    /**
     * adds the add Pose2d X and Y values to the current A and Y values
     * @param add
     */
    public void add( Pose2D add ) {
        X += add.X;
        Y += add.Y;
    }

    /**
     * subtracts the sub Pose2d X and Y values from the current X and Y value
     * @param subtract
     */
    public void sub( Pose2D subtract ) {
        X += subtract.X;
        Y -= subtract.Y;
    }

    /**
     * multiplies the multiply Pose2d X and Y values by the current X and Y value separately
     * @param multiply
     */
    public void mult( Pose2D multiply ) {
        X *= multiply.X;
        Y *= multiply.Y;
    }

    /**
     * divides the current X and Y values by the divide X and Y value separately
     * @param divide
     */
    public void div( Pose2D divide ) {
        X /= divide.X;
        Y /= divide.Y;
    }

    /**
     * @return The distance from this point to 0
     */
    public double getMagnitude() {
        return Math.sqrt(X * X + Y * Y);
    }

    public double dot(Pose2D other) {
        return X * other.X + Y * other.Y;
    }

    /**
     * Rotates the coordinates in an ark
     * @param radians the value that this point is rotated by
     */
    public void rotateRadians(double radians) {
        double x2 = X * cos(radians) - Y * sin(radians);
        double y2 = X * sin(radians) + Y * cos(radians);
        X = x2;
        Y = y2;
    }

    /**
     * @return the angle in radians from 0 counterclockwise to this point
     */
    public double getAngleRadians() {
        return Math.atan2(Y, X);
    }

    /**
     * @param other Vector2d used in angle
     * @return the angle in radians from other clockwise to this point
     */
    public double getAngleRadians(Pose2D other) {
        return Math.atan2(other.Y, other.X) - Math.atan2(Y, X);
    }

    /**
     * Sets magnitude to 1 while maintaining direction
     */
    public void normalize() {
        double magnitude = this.getMagnitude();
        X /= magnitude;
        Y /= magnitude;
    }

    /**
     * @param factor factor that is multiplied by both this X and Y separately
     */
    public void mult(double factor) {
        X *= factor;
        Y *= factor;
    }

    /**
     * @param divisor number that both X and Y are divided by separately
     */
    public void div(double divisor) {
        X /= divisor;
        Y /= divisor;
    }

    /**
     * Makes this Pose2d have the X, Y, and heading values in between -1 and 1 while maintaining direction
     * Useful for when controlling Motors with a pose to ensure that the motor inputs are not greater than 1 or less than -1
     */
    public void capAt1() {
        if (X > 1) {
            heading /= X;
            X /= X;
            Y /= X;
        }
        if (Y > 1) {
            heading /= Y;
            Y /= Y;
            X /= Y;
        }
        if (X < -1) {
            heading /= Math.abs(X);
            X /= Math.abs(X);
            Y /= Math.abs(X);
        }
        if (Y < -1) {
            heading /= Math.abs(Y);
            Y /= Math.abs(Y);
            X /= Math.abs(Y);
        }
        if (heading > 1) {
            heading /= heading;
            X /= heading;
            Y /= heading;
        }
        if (heading < -1) {
            heading /= Math.abs(heading);
            Y /= Math.abs(heading);
            X /= Math.abs(heading);
        }
    }

    //Static methods

    /**
     * Returns a new Pose2d with the coordinates rotated in an ark around origin by specified radians and a magnitude of 1
     * @param radians
     */
    public static Pose2D fromAngleRadians(double radians) {
        return new Pose2D(cos(radians), sin(radians));
    }

    /**
     * Return a new Pose2d with the X and Y as the sum of the X and Y from each of the parameter poses
     */
    public static Pose2D add(Pose2D... add) {
        Pose2D sum = new Pose2D();
        for (Pose2D n : add) {
            sum.X += n.X;
            sum.Y += n.Y;
        }
        return sum;
    }

    /**
     * Returns a new Pose2d with the X, Y and heading values as the differences between the X and Y values from the parameter poses
     */
    public static Pose2D sub(Pose2D sub1, Pose2D sub2) {
        return new Pose2D(sub1.X - sub2.X, sub1.Y - sub2.Y, sub1.heading - sub2.heading );
    }

    /**
     * Returns a new Pose2d with the X and Y as the product of the X and Y values from the parameter poses
     */
    public static Pose2D mult(Pose2D... mult) {
        Pose2D product = new Pose2D(1);
        for (Pose2D n : mult) {
            product.X *= n.X;
            product.Y *= n.Y;
        }
        return product;
    }

    /**
     * Returns a new Pose2d with the X, Y and heading as the quotient of the X and Y values from the parameter poses
     */
    public static Pose2D div(Pose2D div1, Pose2D div2) {
        return new Pose2D(div1.X / div2.X, div1.Y / div2.Y, div1.heading / div2.heading);
    }

    /**
     * Returns a new Pose2d with the X, Y and heading that of the parameter pose plus the add parameter
     */
    public static Pose2D add(Pose2D pose2d, double add) {
        return new Pose2D(pose2d.X + add, pose2d.Y + add, pose2d.heading + add);
    }

    /**
     * Returns a new Pose2d with the X, Y and heading that of the parameter pose minus the sub parameter
     */
    public static Pose2D sub(Pose2D pose2d, double sub) {
        return new Pose2D(pose2d.X - sub, pose2d.Y - sub, pose2d.heading - sub);
    }

    /**
     * Returns a new Pose2d with the X, Y and heading that of the parameter pose divided the divisor parameter
     */
    public static Pose2D div(Pose2D dividend, double divisor) {
        return new Pose2D(dividend.X / divisor, dividend.Y / divisor, dividend.heading / divisor);
    }

    /**
     * Returns a new Pose2d with the X and Y that of the parameter pose times the mult parameter
     */
    public static Pose2D mult(Pose2D a, double b) {
        return new Pose2D(a.X * b, a.Y * b, a.heading * b);
    }

    /**
     * Creates a new Vector2d with the X and Y values that of those in the input Pose2d
     */
    public static Vector2D Vector2d(Pose2D Pose2d){
        return new Vector2D(Pose2d.X, Pose2d.Y);
    }

    /**
     * Returns the magnitude of the parameter pose.
     * This can be visualized as the distance from origin to the X and Y coordinates in the pose
     */
    public static double getMagnitude(Pose2D pose2d) {
        return Math.sqrt(pose2d.X * pose2d.X + pose2d.Y * pose2d.Y);
    }

    /**
     * Returns the midpoint between the two input poses
     */
    public static Pose2D getMidpoint(Pose2D p1, Pose2D p2) {
        return new Pose2D((p1.X + p2.X) / 2, (p1.Y + p2.Y) / 2);
    }
}
