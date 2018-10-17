import QtQuick 2.9
import QtQuick.Controls 2.2
//import QtQuick.Controls 1.0
//import QtQuick.Controls.Styles 1.4
import io.qt.examples.backend 1.0
import QtGraphicalEffects 1.0

ApplicationWindow {
    id: window
    visible: true
    width: 1240
    height: 800

    property bool isConsoleShowing: false
    property bool testbool: false


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
        }


    }

    Rectangle {
        id: header
        width: parent.width
        height: 50
        color: "#bbbbbb"

        Button {
            text: "Show/Hide Console"
            width: parent.width / 3
            height: parent.height
            anchors.horizontalCenter: parent.horizontalCenter

            onClicked:
            {
                if (window.isConsoleShowing)
                {
                    isConsoleShowing = false

                }
                else
                {
                    isConsoleShowing = true
                }

                /*
                if (window.isConsoleShowing){
                    console.log("Popping")
                    stackView.pop()
                    window.isConsoleShowing = false
                }
                else
                {
                    console.log("Pushing")
                    stackView.push(scrollcomponent)
                    window.isConsoleShowing = true
                }
                */
            }
        }

        /*
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
        }*/
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

    Console {
        id:myConsole
        visible: window.isConsoleShowing
        width: parent.width
        height: (parent.height - header.height) / 3
        anchors.bottom: parent.bottom
        displayMessage: backend.debugMessage
    }

    StackView {
        id: stackView
        property alias customHeight : window.height
        property alias isResizing: window.isConsoleShowing
        //anchors.fill: parent
        onCustomHeightChanged: heightResize()
        onIsResizingChanged: heightResize()

        initialItem: csrcn
        anchors.top: header.bottom
        width: parent.width
        height: parent.height - header.height

        function pushFirst(){
            if (stackView.depth < 2) {
                stackView.push(controlView)
            }
        }

        function heightResize()
        {
            if (stackView.isResizing)
            {
                stackView.height = (parent.height - header.height) / 3 * 2
            }
            else
            {
                stackView.height = parent.height - header.height
            }
        }
    }






    Component
    {
        id: csrcn
        ConnectScreen {
            rootText: backend.hostName //qsTr("10.111.144.4")//qsTr("127.0.0.1")
            connectText: backend.connectionText
            onClicked: backend.initConnect()
            onRootTextChanged:
            {
                backend.hostName = rootText
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
            //onStatusClicked: backend.sendGetStatus()
            onInitClicked: backend.sendInit()
            onConnectClicked: backend.sendConnectObject()
            onDisconnectClicked: backend.closeConnect()
            onResetClicked: backend.sendDisconnectObject()

            sysCtrlStatus:  backend.sysCtrlStatus
            objCtrlStatus: backend.objCtrlStatus

        }



    }
    Component {
        id: scrollcomponent
        Console {

        }
    }
}
