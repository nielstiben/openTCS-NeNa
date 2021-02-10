/**
 * Copyright (c) Niels Tiben (nielstiben@outlook.com)
 */
package nl.saxion.nena.opentcs.commadapter.ros2.control_center.control_panel.library;

/**
 * Library for validating user input.
 */
public class InputValidationLib {
    private InputValidationLib() {
        // Empty Constructor
    }

    public static boolean isValidNamespace(String namespace) {
        return namespace.matches("(|(/[a-zA-Z0-9]+))");
    }

    public static boolean isValidDomainId(String namespace) {
        return namespace.matches("(^+?(0|[1-9]\\d*)$)");
    }

    public static String getIsDoubleRegex(){
        return "[-+]?[0-9]*\\.?[0-9]+";
    }
}
