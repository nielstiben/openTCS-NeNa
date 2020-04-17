package nl.saxion.nena.opentcs.commadapter.ros2.control_center.lib;

/**
 * Library for validating user input.
 */
public class InputValidationLib {
    public static boolean isValidDomainId(String domainIdString) {
        if (!isInteger(domainIdString)) {
            return false;
        }

        int domainId = Integer.parseInt(domainIdString);
        return domainId >= 0 && domainId < 300;

    }

    public static boolean isValidCoordinate(String x, String y) {
        return isDouble(x) && isDouble(y);
    }

    private static boolean isDouble(String str) {
        return str.matches(getIsDoubleRegex());
    }

    public static String getIsDoubleRegex(){
        return "[-+]?[0-9]*\\.?[0-9]+";
    }

    private static boolean isInteger(String str) {
        return str.matches("-?(0|[1-9]\\d*)");
    }
}
