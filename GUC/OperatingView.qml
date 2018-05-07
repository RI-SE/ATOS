import QtQuick 2.0
import QtQuick.Controls 2.2
Item {
    id: root
    property int buttonHeight: parent.height * 0.5 - 10
    property int buttonWidth: parent.width * 0.5 - 10
    Grid {
        id: grid

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

}
