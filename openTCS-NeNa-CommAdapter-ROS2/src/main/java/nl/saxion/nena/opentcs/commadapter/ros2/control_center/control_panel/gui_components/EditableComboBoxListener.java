/**
 * Copyright (c) Niels Tiben (nielstiben@outlook.com)
 */
package nl.saxion.nena.opentcs.commadapter.ros2.control_center.control_panel.gui_components;

import javax.swing.*;
import javax.swing.event.DocumentEvent;
import javax.swing.event.DocumentListener;
import java.util.List;
import java.util.Set;
import java.util.function.Function;

import static java.util.Objects.requireNonNull;

/**
 * A validator for the input of the textfield.
 *
 * @author Mustafa Yalciner (Fraunhofer IML)
 */
public class EditableComboBoxListener<E> implements DocumentListener {
  private final JTextField textField;
  private final List<ValidationListener> validationListeners;
  private final Set<E> content;

  /**
   * Returns the string representation for the combo box's selected item.
   */
  private final Function<E, String> representer;

  /**
   * Creates an instance.
   *
   * @param content the elements of the comboBox' dropdownlist.
   * @param validationListeners the listeners to be notified about the validity of the user input.
   * @param textField the textfield with the input to be validated.
   * @param representer Returns the string representation for the combo box's selected item.
   */
  public EditableComboBoxListener(Set<E> content,
                                  List<ValidationListener> validationListeners,
                                  JTextField textField,
                                  Function<E, String> representer) {

    this.content = requireNonNull(content, "content");
    this.validationListeners = requireNonNull(validationListeners, "validationListeners");
    this.textField = requireNonNull(textField, "textField");
    this.representer = requireNonNull(representer, "representer");

  }

  @Override
  public void insertUpdate(DocumentEvent e) {
    validate();
  }

  @Override
  public void removeUpdate(DocumentEvent e) {
    validate();
  }

  @Override
  public void changedUpdate(DocumentEvent e) {
    validate();
  }

  private void notifyValidationListeners(boolean isValid) {
    for (ValidationListener valListener : validationListeners) {
      valListener.onValidityChanged(new ValidationEvent(this, isValid));
    }
  }

  private void validate() {
    if (textField.getText().equals("")) {
      notifyValidationListeners(true);
      return;
    }

    for (E element : content) {
      if (representer.apply(element).equals(textField.getText())) {
        notifyValidationListeners(true);
        return;
      }
    }
    notifyValidationListeners(false);
  }
}
