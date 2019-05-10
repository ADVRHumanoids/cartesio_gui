import QtQuick 2.4

VelocityBarForm {

    property int index: -1

    checkBox.onCheckStateChanged:
    {
        backend.setEnabledAxis(index, checkBox.checked)
    }

}
