package nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.library;

import org.opentcs.data.model.Triple;

import javax.annotation.Nonnull;

/**
 * @author Niels Tiben <nielstiben@outlook.com>
 */
public class UnitConverterLib {
    public static double convertMilimetersToMeters(long millimeters) {
        double millimetersDouble = (double) millimeters;

        return millimetersDouble / 1000;
    }

    @Nonnull
    public static double[] convertTripleToCoordinatesInMeter(@Nonnull Triple triple){
        double x = convertMilimetersToMeters(triple.getX());
        double y = convertMilimetersToMeters(triple.getY());
        double z = convertMilimetersToMeters(triple.getZ());

        return new double[]{x,y,z};
    }

    @Nonnull
    public static Triple convertCoordinatesInMeterToTriple(double x, double y, double z){
        long xInMillimeter = convertMetersToMillimeters(x);
        long yInMillimeter = convertMetersToMillimeters(y);
        long zInMillimeter = convertMetersToMillimeters(z);

        return new Triple(xInMillimeter, yInMillimeter, zInMillimeter);
    }

    private static long convertMetersToMillimeters(double meters) {
        double millimeters = meters * 1000;

        return Math.round(millimeters);
    }
}
