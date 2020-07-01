import QtQuick 2.7
import QtQuick.Window 2.1
import QtQuick.Controls 2.0
import QtQuick.Controls.Styles 1.4
import QtQuick.Extras 1.4
//import CyRobot.Monitor.QNode 1.0
Rectangle  {
    id: root
    visible: true
    width: 1024
    height: 600

    signal openRvizSignal
    signal quitSignal
    signal speed_x_signal(double x)
    signal speed_y_signal(double y)
    //信号处理
    onSpeed_x_signal: {
        valueSource.kph=Math.abs(x*100)
    }
    onSpeed_y_signal: {
        valueSource.kph=Math.abs(y*100)
        valueSource.turnSignal=y>0?Qt.LeftArrow:-1
        valueSource.turnSignal=y<0?Qt.RightArrow:-1
    }
    ValueSource {
        id: valueSource
    }

    SwipeView {
        id: swipeView
        anchors.fill: parent
//        currentIndex: tabBar.currentIndex
        Item {
            id: container
            width: swipeView.width
            height: Math.min(swipeView.width, swipeView.height)

            Rectangle{
                color:"#161616"
                width: parent.width
                height: parent.height
                Row {
                    id: gaugeRow
                    spacing: container.width * 0.02
                    anchors.centerIn: parent

                    TurnIndicator {
                        id: leftIndicator
                        anchors.verticalCenter: parent.verticalCenter
                        width: height
                        height: container.height * 0.1 - gaugeRow.spacing

                        direction: Qt.LeftArrow
                        on: valueSource.turnSignal == Qt.LeftArrow
                    }

                    Item {
                        width: height
                        height: container.height * 0.25 - gaugeRow.spacing
                        anchors.verticalCenter: parent.verticalCenter

                        //油量
                        CircularGauge {
                            id: fuelGauge
                            value: valueSource.fuel
                            maximumValue: 1
                            y: parent.height / 2 - height / 2 - container.height * 0.01
                            width: parent.width
                            height: parent.height * 0.7
                            Text {
                                id: fuelText
                                text: valueSource.fuel+" V"
                                color: "white"
                                font.pixelSize: 10
                                anchors.verticalCenter: parent.verticalCenter
                                anchors.horizontalCenter: parent.horizontalCenter
                            }
                            style: IconGaugeStyle {
                                id: fuelGaugeStyle
                                icon: "qrc:/images/fuel-icon.png"
                                minWarningColor: Qt.rgba(0.5, 0, 0, 1)
                                tickmarkLabel: Text {
                                    color: "white"
                                    visible: styleData.value === 0 || styleData.value === 1
                                    font.pixelSize: fuelGaugeStyle.toPixels(0.225)
                                    text: styleData.value === 0 ? "E" : (styleData.value === 1 ? "F" : "")

                                }
                            }
                        }

                        //温度Page
                        CircularGauge {
                            value: valueSource.temperature
                            maximumValue: 1
                            width: parent.width
                            height: parent.height * 0.7
                            y: parent.height / 2 + container.height * 0.01
                            Text {
                                id: temperatureText
                                text: valueSource.temperature+" ℃"
                                color: "white"
                                font.pixelSize: 10
                                anchors.verticalCenter: parent.verticalCenter
                                anchors.horizontalCenter: parent.horizontalCenter
                            }
                            style: IconGaugeStyle {
                                id: tempGaugeStyle

                                icon: "qrc:/images/temperature-icon.png"
                                maxWarningColor: Qt.rgba(0.5, 0, 0, 1)

                                tickmarkLabel: Text {
                                    color: "white"
                                    visible: styleData.value === 0 || styleData.value === 1
                                    font.pixelSize: tempGaugeStyle.toPixels(0.225)
                                    text: styleData.value === 0 ? "C" : (styleData.value === 1 ? "H" : "")
                                }
                            }
                        }
                    }

                    //速度
                    CircularGauge {
                        id: speedometer
                        value: valueSource.kph
                        anchors.verticalCenter: parent.verticalCenter
                        maximumValue: 150
                        width: height
                        height: container.height * 0.5

                        style: DashboardGaugeStyle {}
                    }

                    //转速
                    CircularGauge {
                        id: tachometer
                        width: height
                        height: container.height * 0.25 - gaugeRow.spacing
                        value: valueSource.rpm
                        maximumValue: 8
                        anchors.verticalCenter: parent.verticalCenter

                        style: TachometerStyle {}
                    }

                    TurnIndicator {
                        id: rightIndicator
                        anchors.verticalCenter: parent.verticalCenter
                        width: height
                        height: container.height * 0.1 - gaugeRow.spacing

                        direction: Qt.RightArrow
                        on: valueSource.turnSignal == Qt.RightArrow
                    }

                }
            }


        }
        Item{

            Button{
               id:quit_btn
               text:"退出主程序"
               onClicked: quitSignal()
            }
            Button{
               text:"Rviz"
               onClicked: openRvizSignal()
               anchors.top: quit_btn.bottom
            }
        }

    }

//    footer: TabBar {
//        id: tabBar
//        currentIndex: swipeView.currentIndex

//        TabButton {
//            text: qsTr("Page 1")
//        }
//        TabButton {
//            text: qsTr("Page 2")
//        }
//    }
}
