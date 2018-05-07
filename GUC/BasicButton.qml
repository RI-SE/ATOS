import QtQuick 2.0

Rectangle { // size controlled by height
    id: root

// public
    property string text: 'text'
    signal clicked(); // onClicked: print('onClicked')

// private
    width: parent.width * 0.7;  height: 50
    color: "#dcd2c8" // default size
    radius:       0.5  * root.height
    border.width: 3
    opacity:      enabled? 1: 0.3 // disabled state

    Text {
        text: root.text
        font.pixelSize: 0.5 * root.height
        anchors.centerIn: parent
    }

    MouseArea {
        anchors.fill: parent
        onPressed:  parent.opacity = 0.5 // down state
        onReleased: parent.opacity = 1
        onCanceled: parent.opacity = 1
        onClicked:  root.clicked() // emit
    }
}
