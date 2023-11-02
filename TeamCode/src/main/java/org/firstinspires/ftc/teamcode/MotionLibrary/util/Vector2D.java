package org.firstinspires.ftc.teamcode.MotionLibrary.util;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

/**
 *  A class that contains an X and Y value.
 *  These can be used as coordinates or as Vectors.
 */
public class Vector2D {

    public double X = 0;
    public double Y = 0;

    /**
     * Creates a new Vector2d with a specified X and Y value
     * @param x
     * @param y
     */
    public Vector2D(double x, double y) {
        X = x;
        Y = y;
    }

    /**
     * Creates a new Vector2d with both the X and Y values as the parameter
     * @param XAndY
     */
    public Vector2D(double XAndY) {
        X = XAndY;
        Y = XAndY;
    }

    /**
     * Creates a new Vector2d with the X and Y values from the Pose2d parameter
     * @param pose2d
     */
    public Vector2D(Pose2D pose2d) {
        X = pose2d.X;
        Y = pose2d.Y;
    }

    /**
     * Creates a new Vector2d with X and Y values 0
     */
    public Vector2D() {}

    /**
     * adds the add Vector2d X and Y values to the current A and Y values
     * @param add
     */
    public void add(Vector2D add) {
        X += add.X;
        Y += add.Y;
    }

    /**
     * adds the add value to both of the current A and Y values
     * @param add
     */
    public void add(double add) {
        X += add;
        Y += add;
    }

    /**
     * subtracts the sub Vector2d X and Y values from the current X and Y value
     * @param sub
     */
    public void sub( Vector2D sub ) {
        X -= sub.X;
        Y -= sub.Y;
    }

    /**
     * multiplies the multiply Vector2d X and Y values by the current X and Y value separately
     * @param multiply
     */
    public void mult( Vector2D multiply ) {
        X *= multiply.X;
        Y *= multiply.Y;
    }

    /**
     * divides the current X and Y values by the divide X and Y value separately
     * @param divide
     */
    public void div( Vector2D divide ) {
        X /= divide.X;
        Y /= divide.Y;
    }

    /**
     * @return The distance from this point to 0
     */
    public double getMagnitude() {
        return Math.sqrt(X * X + Y * Y);
    }

    /**
     *
     * @param other
     */
    public double dot(Vector2D other) {
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
    public double getAngleRadians(Vector2D other) {
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
     * Makes this Vector2d have both the X and Y values in between -1 and 1 while maintaining direction
     * Useful for when controlling Motors with a vector to ensure that the motor inputs are not greater than 1 or less than -1
     */
    public void capAt1() {
        if (X > 1) {
            X /= X;
            Y /= X;
        }
        if (Y > 1) {
            Y /= Y;
            X /= Y;
        }
        if (X < -1) {
            X /= Math.abs(X);
            Y /= Math.abs(X);
        }
        if (Y < -1) {
            Y /= Math.abs(Y);
            X /= Math.abs(Y);
        }
    }

    /**
     * Changes the magnitude of the Vector to the given input while maintaining direction
     * @param scalar
     */
    public void scale(double scalar) {
        this.normalize();
        this.mult(scalar);
    }

    //Static methods

    /**
     * Returns a new Vector2d with the coordinates rotated in an ark around origin by specified radians and a magnitude of 1
     * @param radians
     */
    public static Vector2D fromAngleRadians(double radians) {
        return new Vector2D(cos(radians), sin(radians));
    }


    /**
     * Return a new Vector2d with the X and Y as the sum of the X and Y from each of the parameter vectors
     */
    public static Vector2D add(Vector2D... add) {
        Vector2D sum = new Vector2D();
        for (Vector2D n : add) {
            sum.X += n.X;
            sum.Y += n.Y;
        }
        return sum;
    }

    /**
     * Returns a new Vector2d with the X and Y values as the differences between the X and Y values from the parameter vectors
     */
    public static Vector2D sub(Vector2D sub1, Vector2D sub2) {
        return new Vector2D(sub1.X - sub2.X, sub1.Y - sub2.Y );
    }

    /**
     * Returns a new Vector2d with the X and Y as the product of the X and Y values from the parameter vectors
     */
    public static Vector2D mult(Vector2D... mult) {
        Vector2D product = new Vector2D(1);
        for (Vector2D n : mult) {
            product.X *= n.X;
            product.Y *= n.Y;
        }
        return product;
    }

    /**
     * Returns a new Vector2d with the X and Y as the quotient of the X and Y values from the parameter vectors
     */
    public static Vector2D div(Vector2D div1, Vector2D div2) {
        return new Vector2D(div1.X / div2.X, div1.Y / div2.Y );
    }

    /**
     * Returns a new Vector2d with the X and Y that of the parameter vector plus the add parameter
     */
    public static Vector2D add(Vector2D Vector2d, double add) {
        return new Vector2D(Vector2d.X + add, Vector2d.Y + add);
    }

    /**
     * Returns a new Vector2d with the X and Y that of the parameter vector minus the sub parameter
     */
    public static Vector2D sub(Vector2D Vector2d, double sub) {
        return new Vector2D(Vector2d.X - sub, Vector2d.Y - sub);
    }

    /**
     * Returns a new Vector2d with the X and Y that of the parameter vector divided the divisor parameter
     */
    public static Vector2D div(Vector2D dividend, double divisor) {
        return new Vector2D(dividend.X / divisor, dividend.Y / divisor);
    }

    /**
     * Returns a new Vector2d with the X and Y that of the parameter vector times the mult parameter
     */
    public static Vector2D mult(Vector2D a, double b) {
        return new Vector2D(a.X / b, a.Y / b);
    }

    /**
     * Returns a new Pose2d with the X and Y that of the parameter vector and the heading value equal to 0
     */
    public static Pose2D Pose2d(Vector2D Vector2d){
        return new Pose2D(Vector2d.X, Vector2d.Y);
    }

    /**
     * Returns the magnitude of the parameter vector.
     * This can be visualized as the distance from origin to the X and Y coordinates in the vector
     */
    public static double getMagnitude(Vector2D vector2d) {
        return Math.sqrt(vector2d.X * vector2d.X + vector2d.Y * vector2d.Y);
    }

    /**
     * Returns the angle from the first parameter vector counterclockwise to the second
     */
    public static double getAngle(Vector2D a, Vector2D b) {
        return Vector2D.sub(a, b).getAngleRadians();
    }
}