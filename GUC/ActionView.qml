import QtQuick 2.0

Item {
    id:root

    property int sysCtrlStatus: 0;
    property int objCtrlStatus: 0;

    signal initClicked();
    signal armClicked();
    signal connectClicked();
    signal startClicked();
    signal abortClicked();
    signal disconnectClicked();
    signal statusClicked();

    function isSame(x,y)
    {
        return x === y
    }

    onSysCtrlStatusChanged:
    {
        statusView.sysLed1 = isSame(root.sysCtrlStatus,1)
        statusView.sysLed2 = isSame(root.sysCtrlStatus,2)
        statusView.sysLed3 = isSame(root.sysCtrlStatus,3)
        statusView.sysLed4 = isSame(root.sysCtrlStatus,4)
    }
    onObjCtrlStatusChanged:
    {
        statusView.objLed1 = isSame(root.objCtrlStatus,1)
        statusView.objLed2 = isSame(root.objCtrlStatus,2)
        statusView.objLed3 = isSame(root.objCtrlStatus,3)
        statusView.objLed4 = isSame(root.objCtrlStatus,4)
        statusView.objLed5 = isSame(root.objCtrlStatus,5)
        statusView.objLed6 = isSame(root.objCtrlStatus,6)
    }

    Row {
        id: row
        spacing: 20
        OperatingView {
            id: view1
            width: (root.width - row.spacing) /2
            height: root.height
            onInitClicked: root.initClicked()
            onArmClicked: root.armClicked()
            onConnectClicked: root.connectClicked()
            onStartClicked: root.startClicked()
            onAbortClicked: root.abortClicked()
            onDisconnectClicked: root.disconnectClicked()
            onStatusClicked: root.statusClicked()
        }

        StatusView {
            id: statusView
            width: (root.width - row.spacing) /2
            height: root.height


        }
    }
}
