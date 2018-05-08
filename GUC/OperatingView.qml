import QtQuick 2.0
import QtQuick.Controls 2.2

Grid {
    id: root
    property int buttonHeight: height * 0.5 - 10
    property int buttonWidth: width * 0.5 - 10

    spacing: 10
    layoutDirection: Qt.LeftToRight
    flow: Grid.LeftToRight
    rows: 2
    columns: 2
    Button {
        id: armButton
        text: qsTr("ARM")
        height: root.buttonHeight
        width: root.buttonWidth
    }

    Button {
        id: disarmButton
        text: qsTr("DISARM")
        height: root.buttonHeight
        width: root.buttonWidth
    }

    Button {
        id: startButton
        text: qsTr("START")
        height: root.buttonHeight
        width: root.buttonWidth
    }

    Button {
        id: abortButton
        text: qsTr("STOP")
        height: root.buttonHeight
        width: root.buttonWidth
    }
}


