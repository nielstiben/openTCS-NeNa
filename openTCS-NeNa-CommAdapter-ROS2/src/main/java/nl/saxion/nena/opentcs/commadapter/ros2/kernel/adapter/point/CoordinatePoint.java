package nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.point;

import lombok.Getter;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.library.UnitConverterLib;
import org.opentcs.data.model.Point;
import org.opentcs.data.model.Triple;

/**
 * Fictional point,used to navigate a AGV to a coordinate. CoordinatePoint instances are not stored in a plant model.
 *
 * @author Niels Tiben
 */
@Getter
public class CoordinatePoint extends Point {
    private final Triple position;

    public CoordinatePoint(Triple coordinate) {
        super(String.format(
                "coordinate (%.2f, %.2f)",
                UnitConverterLib.convertMilimetersToMeters(coordinate.getX()),
                UnitConverterLib.convertMilimetersToMeters(coordinate.getY())
        ));
        this.position = coordinate;
    }
}
