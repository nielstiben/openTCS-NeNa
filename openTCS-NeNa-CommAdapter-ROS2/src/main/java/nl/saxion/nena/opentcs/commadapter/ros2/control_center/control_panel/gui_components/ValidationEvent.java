/**
 * Copyright (c) Niels Tiben (nielstiben@outlook.com)
 */
package nl.saxion.nena.opentcs.commadapter.ros2.control_center.control_panel.gui_components;

import lombok.Getter;

import java.util.EventObject;

/**
 * An event holding a single boolean variable indicating if something is valid.
 *
 * @author Niels Tiben
 */
public class ValidationEvent extends EventObject {
    @Getter
    private final boolean isValid;

    public ValidationEvent(Object source, boolean isValid) {
        super(source);
        this.isValid = isValid;
    }
}
