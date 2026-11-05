package org.firstinspires.ftc.teamcode.utilities;

import java.util.Arrays;

/**
 * Mathematical helper class representing a single-variable polynomial function.
 * Stores polynomial coefficients and provides an evaluation method using Horner's scheme.
 * 
 * Polynomial format: f(x) = c_0 + c_1 * x + c_2 * x^2 + ... + c_n * x^n
 */
public class Polynomial {
    private final double[] coefficients;

    /**
     * Constructs a polynomial with specified coefficients ordered from lowest degree (c0) to highest (cn).
     *
     * @param coefficients Coefficients array [c0, c1, c2, ..., cn]
     */
    public Polynomial(double... coefficients) {
        if (coefficients == null || coefficients.length == 0) {
            this.coefficients = new double[]{0.0};
        } else {
            this.coefficients = Arrays.copyOf(coefficients, coefficients.length);
        }
    }

    /**
     * Evaluates the polynomial at a given value x using Horner's method.
     *
     * @param x Input variable (e.g., target distance)
     * @return Calculated polynomial value f(x)
     */
    public double evaluate(double x) {
        double result = 0.0;
        for (int i = coefficients.length - 1; i >= 0; i--) {
            result = result * x + coefficients[i];
        }
        return result;
    }

    /**
     * Factory method for creating a polynomial.
     *
     * @param coefficients Coefficients [c0, c1, ...]
     * @return Polynomial instance
     */
    public static Polynomial of(double... coefficients) {
        return new Polynomial(coefficients);
    }

    /**
     * Factory method for a constant polynomial f(x) = c0.
     */
    public static Polynomial constant(double c0) {
        return new Polynomial(c0);
    }

    /**
     * Factory method for a linear polynomial f(x) = c0 + c1 * x.
     */
    public static Polynomial linear(double c0, double c1) {
        return new Polynomial(c0, c1);
    }

    /**
     * Returns the degree of the polynomial.
     */
    public int getDegree() {
        return coefficients.length - 1;
    }

    /**
     * Gets a copy of the coefficients array.
     */
    public double[] getCoefficients() {
        return Arrays.copyOf(coefficients, coefficients.length);
    }

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder("f(x) = ");
        for (int i = 0; i < coefficients.length; i++) {
            if (i > 0) sb.append(" + ");
            sb.append(coefficients[i]);
            if (i == 1) sb.append("*x");
            else if (i > 1) sb.append("*x^").append(i);
        }
        return sb.toString();
    }
}
