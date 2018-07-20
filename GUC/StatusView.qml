import QtQuick 2.0
import QtQuick.Controls 2.2
import QtQuick.Extras 1.4
Item {
    id: root

    property bool sysLed1: false
    property bool sysLed2: false
    property bool sysLed3: false
    property bool sysLed4: false

    property bool objLed1: false
    property bool objLed2: false
    property bool objLed3: false
    property bool objLed4: false
    property bool objLed5: false
    property bool objLed6: false

    property int nrColElems: 13
    property int elemHeight: root.height / nrColElems

    Column {
        Text {
            id: sysname
            text: qsTr("System Control State")
            font.bold: true
            font.underline: true
            height: elemHeight
            font.pixelSize: 0.5 * elemHeight
        }
        LEDandText {
            id: sysi1
            height: elemHeight
            text: qsTr("INITIALIZED")
            lightOn: root.sysLed1
        }
        LEDandText {
            id: sysi2
            height: elemHeight
            text: qsTr("IDLE")
            lightOn: root.sysLed2
        }
        LEDandText {
            id: sysi3
            height: elemHeight
            text: qsTr("INWORK")
            lightOn: root.sysLed3
        }
        LEDandText {
            id: sysi4
            height: elemHeight
            text: qsTr("FAIL")
            lightOn: root.sysLed4
        }

        Rectangle {
            width: root.width
            height: elemHeight
        }
        Text {
            id: objname
            text: qsTr("Object Control State")
            font.bold: true
            font.underline: true
            height: elemHeight
            font.pixelSize: 0.5 * elemHeight
        }
        LEDandText {
            id: obji1
            height: elemHeight
            text: qsTr("IDLE")
            lightOn: root.objLed1
        }
        LEDandText {
            id: obji2
            height: elemHeight
            text: qsTr("INITIALIZED")
            lightOn: root.objLed2
        }
        LEDandText {
            id: obji3
            height: elemHeight
            text: qsTr("CONNECTED")
            lightOn: root.objLed3
        }
        LEDandText {
            id: obji4
            height: elemHeight
            text: qsTr("ARMED")
            lightOn: root.objLed4
        }
        LEDandText {
            id: obji5
            height: elemHeight
            text: qsTr("RUNNING")
            lightOn: root.objLed5
        }
        LEDandText {
            id: obji6
            height: elemHeight
            text: qsTr("ERROR")
            lightOn: root.objLed6
            color: "red"
        }
    }
}


