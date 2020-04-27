package nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.library;

import lombok.Setter;
import org.opentcs.data.model.Triple;

import javax.annotation.Nonnull;

/**
 * @author Niels Tiben
 */
public class ScaleCorrector {
    private static ScaleCorrector scaleCorrectorInstance;

    @Setter
    private double scale;

    public static ScaleCorrector getInstance() {
        if (scaleCorrectorInstance == null) {
            scaleCorrectorInstance = new ScaleCorrector();
        }
        return scaleCorrectorInstance;
    }

    public double[] scaleCoordinatesForDevice(@Nonnull double[] coordinates) {
        assert coordinates.length == 3;

        double xScaled = scaleDoubleForDevice(coordinates[0]);
        double yScaled = scaleDoubleForDevice(coordinates[1]);
        double zScaled = scaleDoubleForDevice(coordinates[2]);

        return new double[]{xScaled, yScaled, zScaled};
    }

    public Triple scaleTripleForFleetManager(@Nonnull Triple triple) {
        long[] coordinates = new long[]{triple.getX(), triple.getY(), triple.getZ()};

        long xScaled = scaleLongForFleetManager(coordinates[0]);
        long yScaled = scaleLongForFleetManager(coordinates[1]);
        long zScaled = scaleLongForFleetManager(coordinates[2]);

        return new Triple(xScaled, yScaled, zScaled);
    }

    private double scaleDoubleForDevice(double fromFleetManager) {
        return fromFleetManager * this.scale;
    }

    private long scaleLongForFleetManager(long fromDevice) {
        double fromDeviceDouble = (double) fromDevice;

        return Math.round(fromDeviceDouble / this.scale);
    }
}
