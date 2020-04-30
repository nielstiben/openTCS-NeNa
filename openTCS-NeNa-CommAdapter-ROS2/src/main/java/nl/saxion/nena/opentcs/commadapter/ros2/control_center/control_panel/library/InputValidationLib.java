package nl.saxion.nena.opentcs.commadapter.ros2.control_center.control_panel.library;

/**
 * Library for validating user input.
 */
public class InputValidationLib {
    public static boolean isValidNamespace(String namespace) {
        return namespace.matches("(|(/[a-zA-Z0-9]+))");
    }

    public static String getIsDoubleRegex(){
        return "[-+]?[0-9]*\\.?[0-9]+";
    }

    private static boolean isInteger(String str) {
        return str.matches("-?(0|[1-9]\\d*)");
    }
}
