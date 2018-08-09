import QtQuick 2.0
import QtQuick.Controls 2.2
Item {
    id: root
    property alias rootText: input.text
    property string connectText: ""
    signal clicked();
    //anchors.verticalCenter: parent.verticalCenter
    //anchors.horizontalCenter: parent.horizontalCenter



    function setIPstatus(status) {

        if (status === 0)
            input.color = "red"
        else
            input.color = "green"
    }

    Rectangle {
        color: "white"
        id: conBackground
        anchors.fill: parent


        Rectangle {
            id: rectangle1
            color: "#e3e0e0"
            radius: 55
            border.color: "#e3e0e0"
            width: 360
            height: 300
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.verticalCenter: parent.verticalCenter

            BasicButton {
                //y: 136
                id: conn
                text: "Connect..."
                anchors.horizontalCenterOffset: 0
                anchors.horizontalCenter: parent.horizontalCenter
                anchors.verticalCenter: parent.verticalCenter
                onClicked:
                {
                    root.clicked()
                    input.deselect()
                }
            }

            TextField {
                //property int isValid : 0
                id: input
                width: parent.width * 0.7
                height: 40
                font.bold: true
                font.pointSize: 18
                horizontalAlignment: Text.AlignHCenter
                anchors.bottom: conn.top
                anchors.bottomMargin: 30
                anchors.horizontalCenterOffset: 0
                anchors.horizontalCenter: parent.horizontalCenter
                placeholderText: qsTr("IP Address")
                onAccepted: deselect()
            }

            Label {
                id: label
                x: 171
                text: root.connectText
                padding: 0
                topPadding: 0
                anchors.horizontalCenterOffset: 0
                horizontalAlignment: Text.AlignHCenter
                anchors.horizontalCenter: input.horizontalCenter
                anchors.top: conn.bottom
                anchors.topMargin: 30
            }

        }
    }
}
