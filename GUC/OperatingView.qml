import QtQuick 2.0
import QtQuick.Controls 2.2

Column {
    id: root
    property int buttons: 4
    property int buttonHeight: (height - (buttons - 1) * 10)/buttons
    property int buttonWidth: width


    signal armClicked();
    signal disarmClicked();
    signal startClicked();
    signal abortClicked();

    spacing: 10


    Button {
        id: armButton
        text: qsTr("ARM")
        height: root.buttonHeight
        width: root.buttonWidth
        onClicked: root.armClicked();
    }


    Button {
        id: disarmButton
        text: qsTr("DISARM")
        height: root.buttonHeight
        width: root.buttonWidth
        onClicked: root.disarmClicked()
    }


    Button {
        id: startButton
        text: qsTr("START")
        height: root.buttonHeight
        width: root.buttonWidth
        onClicked: root.startClicked()
    }


    Button {
        id: abortButton
        text: qsTr("ABORT")
        height: root.buttonHeight
        width: root.buttonWidth
        onClicked: root.abortClicked()
    }

    Column {
        id: column
        width: 200
        height: 400
    }
}


