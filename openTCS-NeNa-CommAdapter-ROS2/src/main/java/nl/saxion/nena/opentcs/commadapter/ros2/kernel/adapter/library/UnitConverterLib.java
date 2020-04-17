package nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.library;

/**
 * @author Niels Tiben <nielstiben@outlook.com>
 */
public class UnitConverterLib {
    public static double convertMilimetersToMeters(long milimeters) {
        double milimetersDouble = (double) milimeters;

        return milimetersDouble / 1000;
    }
}
