import QtQuick 2.4
import QtQuick.Controls 2.3
import QtQuick.Layouts 1.0

Item {
    id: element
    width: 40
    height: 230
    property alias signedBar: signedBar
    property alias textField: textField
    property alias checkBox: checkBox
    property alias label: label

    Label {
        id: label
        x: 14
        y: 8
        text: "RX"
        anchors.horizontalCenter: parent.horizontalCenter
        verticalAlignment: Text.AlignVCenter
        horizontalAlignment: Text.AlignHCenter
        width: parent.width
    }

    CheckBox {
        id: checkBox
        x: 10
        y: 187
        text: qsTr("")
        checked: true
        display: AbstractButton.IconOnly
        anchors.horizontalCenter: parent.horizontalCenter
    }

    TextField {
        id: textField
        x: 3
        y: 141
        width: signedBar.width
        height: 40
        text: signedBar.value.toFixed(2)
        font.pointSize: 10
        anchors.horizontalCenter: parent.horizontalCenter
        readOnly: true
        enabled: false
    }

    SignedBar {
        id: signedBar
        x: 7
        y: 26
        width: parent.width
        height: 108
        value: 1
        anchors.horizontalCenter: parent.horizontalCenter
    }
}




/*##^## Designer {
    D{i:0;height:320;width:64}
}
 ##^##*/
