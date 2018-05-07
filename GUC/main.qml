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


    BackEnd {
        id: backend
        onEnterStartScreen:
        {
            console.log("Start Screen connected")
            stackView.push(buttonView)
        }
        onEnterConnectionScreen:
        {
            console.log("Wanting to reach connection screen.");
            stackView.pop()
        }
    }

    StackView {
        id: stackView
        anchors.fill: parent
        initialItem: csrcn
    }

    Component
    {
        id: csrcn
        ConnectScreen {
            connectText: backend.connectionText
            rootText: qsTr("10.111.144.28")

            onClicked: backend.initConnect()
            onRootTextChanged:
            {
                backend.setHostName(rootText)
                setIPstatus(backend.addressValid(rootText))
            }
            anchors.verticalCenter: parent.verticalCenter
            anchors.horizontalCenter: parent.horizontalCenter
        }
    }
    Component
    {
        id: rect
        RoundedSquare {
            anchors.verticalCenter: parent.verticalCenter
            anchors.horizontalCenter: parent.horizontalCenter
            onClicked: stackView.pop()
        }
    }
    Component {
        id: buttonView
        OperatingView {
        }
    }
}
