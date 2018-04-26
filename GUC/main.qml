import QtQuick 2.9
import QtQuick.Controls 2.2
import io.qt.examples.backend 1.0

ApplicationWindow {
    id: window
    visible: true
    width: 640
    height: 480
    //title: qsTr("GUC")

    BackEnd {
        id: backend
    }

    Rectangle {
        property string orgColor: "#21e736"
        id: rectangle
        x: 220
        y: 140
        width: 200
        height: 200
        color: orgColor
        anchors.horizontalCenterOffset: 0
        border.width: 7
        border.color: "#0e0808"
        anchors.horizontalCenter: parent.horizontalCenter



        MouseArea {
            id: startArea
            anchors.fill: parent
            onPressed: {
                parent.color = "#9cea95"
            }
            onReleased: {
                parent.color = parent.orgColor
            }

            Text {
                id: text1
                x: 88
                y: 94
                text: qsTr("Start Test")
                font.bold: true
                fontSizeMode: Text.FixedSize
                verticalAlignment: Text.AlignVCenter
                horizontalAlignment: Text.AlignHCenter
                anchors.verticalCenter: parent.verticalCenter
                anchors.horizontalCenter: parent.horizontalCenter
                font.pixelSize: 20
            }
        }
    }

    TextField {
        x: 220
        y: 346
        text: backend.userName
        anchors.horizontalCenter: rectangle.horizontalCenter
        placeholderText: qsTr("User Name")
        onTextChanged: backend.userName = text
    }

}
