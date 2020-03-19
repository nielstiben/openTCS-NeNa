/**
 * Copyright (c) Fraunhofer IML
 */
package nl.saxion.nena.opentcs.common.telegrams;

import static org.opentcs.util.Assertions.checkArgument;

/**
 * A counter that is bounded below by its initial value and bounded above by its maximum value.
 * If the counter exceeds the maximum value it resets to the initial value.
 *
 * @author Martin Grzenia (Fraunhofer IML)
 */
public class BoundedCounter {

  /**
   * The maximum value for an unsigned 16 bit integer.
   */
  public static final int UINT16_MAX_VALUE = 65535;
  /**
   * The initial counter value.
   * Exceeding the {@code maxValue} resets the counter to this value.
   */
  private final int initialValue;
  /**
   * The maximum counter value.
   * Exceeding this value will reset the the counter to the {@code initialValue}.
   */
  private final int maxValue;
  /**
   * The counter's current value.
   */
  private int counterValue;

  /**
   * Creates a new instance.
   *
   * @param initialValue The initial counter value.
   * @param maxValue The maximum counter value.
   */
  public BoundedCounter(int initialValue, int maxValue) {
    checkArgument(initialValue < maxValue,
                  "initialValue has to lower than maxValue: %d < %d",
                  initialValue,
                  maxValue);
    this.initialValue = initialValue;
    this.maxValue = maxValue;
    this.counterValue = initialValue;
  }

  /**
   * Returns the current counter value and increments it afterwards.
   *
   * @return The current counter value.
   */
  public int getAndIncrement() {
    int currValue = counterValue;
    counterValue++;
    if (counterValue > maxValue) {
      counterValue = initialValue;
    }
    return currValue;
  }
}
