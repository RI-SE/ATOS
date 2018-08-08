import QtQuick 2.0

Rectangle{
    id: root
    property bool isLightOn: false
    property int circRad: 50

    width: circRad
    height: circRad
    border.width: 3
    radius: 0.5 * root.width
    color: isLightOn ? "green" : "black"
    /*
    onIsLightOnChanged:
    {
        console.log("LIGHT ON CHANGED")
        if(isLightOn)
        {
            color: "green"
            console.log(color)
        }
        else
        {
            color: "black"
            console.log(color)
        }
    }
    */
}

