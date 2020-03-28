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
        if (domainId < 0 || domainId >= 300) {
            return false;
        }

        return true;

    }

    private static boolean isInteger(String str) {
        return str.matches("-?(0|[1-9]\\d*)");
    }
}
