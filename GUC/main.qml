import QtQuick 2.9
import QtQuick.Controls 2.2
import io.qt.examples.backend 1.0
import QtGraphicalEffects 1.0

ApplicationWindow {
    id: window
    visible: true
    width: 640
    height: 480
    //title: qsTr("GUC")

    Rectangle {
        id: rectangle1
        x: 220
        y: 140
        width: 372
        height: 322
        color: "#e3e0e0"
        radius: 55
        anchors.verticalCenter: parent.verticalCenter
        anchors.horizontalCenter: parent.horizontalCenter
        border.color: "#e3e0e0"
/*
        Rectangle {
            property string orgColor: "#21e736"
            id: rectangle
            x: 86
            y: 124
            width: 124
            height: 120
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
        } */


        BasicButton {
            //y: 136
            id: conn
            text: "Connect..."
            anchors.horizontalCenterOffset: 0
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.verticalCenter: parent.verticalCenter
            onClicked: backend.connect()
        }

        TextField {
            width: 250
            height: 40
            text: backend.userName
            font.bold: true
            font.pointSize: 11
            horizontalAlignment: Text.AlignHCenter
            anchors.bottom: conn.top
            anchors.bottomMargin: 30
            anchors.horizontalCenterOffset: 0
            anchors.horizontalCenter: parent.horizontalCenter
            placeholderText: qsTr("IP Address")
            onTextChanged: backend.userName = text
        }

        Text {
            id: text1
            x: 175
            text: qsTr("Text")
            anchors.horizontalCenterOffset: 0
            anchors.top: conn.bottom
            anchors.topMargin: 30
            anchors.horizontalCenter: parent.horizontalCenter
            visible: false
            font.pixelSize: 20
        }
    }

    BackEnd {
        id: backend
    }





}
