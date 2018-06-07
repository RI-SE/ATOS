import QtQuick 2.0
import QtQuick.Controls 2.2

Column {
    id: root

    spacing: 10
    property int buttons: 5
    property int buttonHeight: (height - (buttons - 1) * spacing)/buttons
    property int buttonWidth: width


    signal initClicked();
    signal armClicked();
    signal connectClicked();
    signal startClicked();
    signal abortClicked();
    signal disconnectClicked();
    signal statusClicked();


    BasicButton {
        id: initButton
        text: qsTr("INIT")
        height: root.buttonHeight
        width: root.buttonWidth
        onClicked: root.initClicked();
    }

/*
    BasicButton {
        id: connectButton
        text: qsTr("CONNECT OBJECTS")
        height: root.buttonHeight
        width: root.buttonWidth
        onClicked: root.connectClicked()
    }*/

    BasicButton {
        id: armButton
        text: qsTr("ARM")
        height: root.buttonHeight
        width: root.buttonWidth
        onClicked: root.armClicked();
    }

    BasicButton {
        id: startButton
        text: qsTr("START")
        height: root.buttonHeight
        width: root.buttonWidth
        onClicked: root.startClicked()
        color: "green"
    }


    BasicButton {
        id: abortButton
        text: qsTr("ABORT")
        height: root.buttonHeight
        width: root.buttonWidth
        onClicked: root.abortClicked()
        color: "red"
    }
/*
    BasicButton {
        id: statusButton
        text: qsTr("GetStatus()")
        height: root.buttonHeight
        width: root.buttonWidth
        onClicked: root.statusClicked()
        color: "orange"
    }
*/
    BasicButton {
        id: disconnectButton
        text: qsTr("Disconnect...")
        height: root.buttonHeight
        width: root.buttonWidth
        onClicked: root.disconnectClicked()
    }
}


