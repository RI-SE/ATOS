import QtQuick 2.9
import QtQuick.Controls 2.2

Item {
    id:root

    property string displayMessage: ""

    property real vPos: 1.0
    onDisplayMessageChanged:
    {
        textArea.append(displayMessage)
    }
    Rectangle
    {
        width: root.width
        height: root.height
        color: "black"

        ScrollView {
            id:scrollview
            anchors.fill: parent

            TextArea {
                id: textArea
                //text: backend.connectionText
                //font.capitalization: Font.AllUppercase
                verticalAlignment: Text.AlignBottom
                color: "white"
                //text: root.displayMessage
            }
        }

    }

}
