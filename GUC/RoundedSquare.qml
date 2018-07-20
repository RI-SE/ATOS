import QtQuick 2.0

Item {
    id: root
    signal clicked();

    width: 360
    height: 300
    Rectangle {
        width: root.width
        height: root.height
        color: "red"
        radius: 25

        MouseArea {
            id: mouseArea
            anchors.fill: parent
            onClicked: root.clicked()
        }

    }
}
