import QtQuick 2.0
import QtQuick.Controls 2.2
//import QtQuick.Extras 1.4

Item {
    id: root
    property string text: "Text"
    property string color: "green"
    property bool lightOn: false
    width: 300
    height: 150
    onLightOnChanged: console.log("LED changed")
    /*
    StatusIndicator {
        id: statusIndicator
        width: root.height * 0.8
        height: root.height * 0.8
        color: root.color
        anchors.verticalCenter: parent.verticalCenter

        active: lightOn
    }*/
    MyLED{
        id:statusIndicator
        circRad: root.height * 0.8
        isLightOn: lightOn
        anchors.verticalCenter: parent.verticalCenter
    }

    Text {
        width: parent.width - statusIndicator.width
        text: root.text
        font.pixelSize: root.height * 0.4
        anchors.left: statusIndicator.right
        anchors.verticalCenter: statusIndicator.verticalCenter
    }

}
