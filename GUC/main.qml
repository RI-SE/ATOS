import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Controls.Styles 1.4
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
            stackView.pushFirst()
        }
        onEnterConnectionScreen:
        {
            stackView.pop()
        }
        onNewDebugMessage: {
            //console.log(debugText)
            textArea.append(debugText)
        }

    }

    Rectangle {
        id: header
        width: parent.width
        height: 50
        color: "green"

        Button {
            id: back
            x: 8
            y: 5
            text: qsTr("<")
            onClicked: stackView.pop()
        }

        Button {
            id: forward
            x: 532
            y: 5
            text: qsTr(">")
            onClicked: stackView.pushFirst()
        }
    }

    Rectangle {
        id: footer
        width: parent.width
        height: 100
        color: "black"
        anchors.bottom: parent.bottom

        ScrollView {
            id: view
            anchors.fill: parent
            TextArea {
                id: textArea
                //text: backend.connectionText
                //font.capitalization: Font.AllUppercase
                verticalAlignment: Text.AlignBottom
                anchors.fill: parent
                color: "white"
            }
        }
    }



    StackView {
        id: stackView
        //anchors.fill: parent
        initialItem: csrcn
        anchors.top: header.bottom
        anchors.bottom: footer.top
        width: parent.width
        //height: parent.height - header.height

        function pushFirst(){
            if (stackView.depth < 2) {
                stackView.push(buttonView)
            }
        }


    }



    Component
    {
        id: csrcn
        ConnectScreen {
            connectText: backend.connectionText
            rootText: qsTr("10.111.144.21")

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

    Component {
        id: buttonView
        OperatingView {
            anchors.horizontalCenter: window.horizontalCenter
            onArmClicked: backend.sendArmToHost()
            onDisarmClicked: backend.sendDisarmToHost()
            onStartClicked: backend.sendStartToHost(1000)
            onAbortClicked: backend.sendAbortToHost()
        }
    }
}
