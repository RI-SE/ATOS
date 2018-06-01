import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Controls.Styles 1.4
import io.qt.examples.backend 1.0
import QtGraphicalEffects 1.0

ApplicationWindow {
    id: window
    visible: true
    width: 1240
    height: 800
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
            //textArea.append(debugText)
        }


    }

    Rectangle {
        id: header
        width: parent.width
        height: 50
        color: "#bbbbbb"

        Button {
            id: back
            y: 5
            text: qsTr("<")
            anchors.left: parent.left
            anchors.leftMargin: 8
            onClicked: stackView.pop()
        }

        Button {
            id: forward
            y: 5
            anchors.right: parent.right
            anchors.rightMargin: 8
            text: qsTr(">")
            onClicked: stackView.pushFirst()
        }
    }
/*
    Rectangle {
        id: footer
        width: parent.width
        height: 300
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
    } */



    StackView {
        id: stackView
        //anchors.fill: parent
        initialItem: csrcn
        anchors.top: header.bottom
        anchors.bottom: parent.bottom
        width: parent.width
        //height: parent.height - header.height

        function pushFirst(){
            if (stackView.depth < 2) {
                stackView.push(controlView)
            }
        }


    }



    Component
    {
        id: csrcn
        ConnectScreen {
            connectText: backend.connectionText
            rootText: qsTr("10.130.0.10")
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
        id: controlView
        ActionView {
            onArmClicked: backend.sendArmToHost()
            onStartClicked: backend.sendStartToHost(1000)
            onAbortClicked: backend.sendAbortToHost()
            onStatusClicked: backend.sendGetStatus()
            onInitClicked: backend.sendInit()
            onConnectClicked: backend.sendConnectObject()
            onDisconnectClicked: backend.sendDisconnectObject()

            sysCtrlStatus:  backend.sysCtrlStatus
            objCtrlStatus: backend.objCtrlStatus


        }
    }
}
