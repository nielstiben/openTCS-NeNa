package nl.saxion.nena.opentcs.commadapter.ros2.control_center.control_panel.gui_components;

import java.util.EventListener;

/**
 * A listener interface for {@link ValidationEvent ValidationEvents}.
 *
 * @author Niels Tiben
 */
public interface ValidationListener extends EventListener {

  void onValidityChanged(ValidationEvent e);
}
