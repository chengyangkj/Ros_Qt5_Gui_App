import QtQuick 2.0
import QtQuick.Window 2.0
import QtQuick.Controls 1.0
import QRoboMap 1.0
Window {
    id: window
    visible: true
    width: 800
    height: 800
    title: qsTr("MainWindow")
    MouseArea{
        id:mouseWheel
        width: parent.width
        height: parent.height
        anchors.fill: parent
        hoverEnabled: true
        onWheel: {
            if(wheel.angleDelta.y>0){
                roboMap_.setMax()
            }
            else{
                roboMap_.setMin()
            }
        }
        onMouseXChanged: {
           console.log(mouse.x)
        }
        onPressAndHold: {
            console.log(mouse.x)
        }

    }
    Rectangle{
        id:content
        border.color: "red"
        width: parent.width
        height: parent.height
        RoboMap{
            id:roboMap_
            anchors.fill: parent
            height:parent.height
            width:parent.width
        }
    }



    Button{
        text: "click"
        onClicked: roboMap.get_version()
    }
}
