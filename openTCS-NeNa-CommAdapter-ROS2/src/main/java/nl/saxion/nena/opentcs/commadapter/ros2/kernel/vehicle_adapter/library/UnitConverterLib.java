/**
 * Copyright (c) Niels Tiben (nielstiben@outlook.com)
 */
package nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.library;

import org.opentcs.data.model.Triple;
import us.ihmc.euclid.tuple4D.Quaternion;

import javax.annotation.Nonnull;

/**
 * Library class for converting distance and rotation units.
 *
 * @author Niels Tiben
 */
public abstract class UnitConverterLib {
    public static double convertMillimetersToMeters(long millimeters) {
        double millimetersDouble = (double) millimeters;

        return millimetersDouble / 1000;
    }

    public static double[] convertTripleToCoordinatesInMeter(@Nonnull Triple triple) {
        double x = convertMillimetersToMeters(triple.getX());
        double y = convertMillimetersToMeters(triple.getY());
        double z = convertMillimetersToMeters(triple.getZ());

        return new double[]{x, y, z};
    }

    public static Triple convertCoordinatesInMeterToTriple(double x, double y, double z) {
        long xInMillimeter = convertMetersToMillimeters(x);
        long yInMillimeter = convertMetersToMillimeters(y);
        long zInMillimeter = convertMetersToMillimeters(z);

        return new Triple(xInMillimeter, yInMillimeter, zInMillimeter);
    }

    private static long convertMetersToMillimeters(double meters) {
        double millimeters = meters * 1000;

        return Math.round(millimeters);
    }

    public static double quaternionToAngleDegree(@Nonnull Quaternion quaternion) {
        // Convert quaternion to angle radians
        double angleRad = 2.0 * Math.acos(quaternion.getZ());

        // Inverse angle radian
        angleRad = angleRad * -1;

        // Convert angle radians to angle degrees
        double angleDegree = Math.toDegrees(angleRad);

        // Add 180° to comply with OpenTCS way of storing orientations (n=90°; e=0°; s=270°; w=180°)
        angleDegree = angleDegree + 180;

        // Normalize degree between 0° and 360°
        angleDegree = angleDegree % 360;
        if (angleDegree < 0) {
            angleDegree += 360;
        }

        return angleDegree;
    }
}
