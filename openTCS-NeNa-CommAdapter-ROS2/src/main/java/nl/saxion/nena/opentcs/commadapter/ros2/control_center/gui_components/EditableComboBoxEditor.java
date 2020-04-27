package nl.saxion.nena.opentcs.commadapter.ros2.control_center.gui_components;

import javax.annotation.Nonnull;
import javax.swing.*;
import javax.swing.event.ListDataEvent;
import javax.swing.event.ListDataListener;
import javax.swing.plaf.basic.BasicComboBoxEditor;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.function.Function;

/**
 * A simple editor for editable combo boxes.
 *
 * @author Niels Tiben
 */
public class EditableComboBoxEditor<E> extends BasicComboBoxEditor implements ListDataListener {
    private final Set<E> comboBoxSet = new HashSet<>();
    private final JComboBox<E> comboBox;
    private final Function<E, String> selectedItemRepresentation;

    //================================================================================
    // Constructor
    //================================================================================
    public EditableComboBoxEditor(List<ValidationListener> validationListeners,
                                  @Nonnull JComboBox<E> comboBox,
                                  @Nonnull Function<E, String> selectedItemRepresentation) {
        this.comboBox = comboBox;
        this.selectedItemRepresentation = selectedItemRepresentation;
        editor.getDocument().addDocumentListener(
                new EditableComboBoxListener<>(this.comboBoxSet, validationListeners, editor, selectedItemRepresentation)
        );
    }

    //================================================================================
    // Override methods
    //================================================================================
    @Override
    public void intervalAdded(ListDataEvent e) {
        loadContent();
    }

    @Override
    public void intervalRemoved(ListDataEvent e) {
        loadContent();
    }

    @Override
    public void contentsChanged(ListDataEvent e) {
        loadContent();
    }

    @Override
    public Object getItem() {
        // if the panel tries to capture the input, this method guarantees that
        for (E p : comboBoxSet) {
            if (selectedItemRepresentation.apply(p).equals(editor.getText())) {
                return p;
            }
        }
        return null;
    }

    @Override
    @SuppressWarnings("unchecked")
    public void setItem(Object anObject) {
        editor.setText(selectedItemRepresentation.apply((E) anObject));
    }

    //================================================================================
    // Private methods
    //================================================================================
    private void loadContent() {
        //get the current comboBoxModel and add the model elements to content
        ComboBoxModel<E> model = comboBox.getModel();

        for (int i = 0; i < model.getSize(); i++) {
            comboBoxSet.add(model.getElementAt(i));
        }
    }
}
