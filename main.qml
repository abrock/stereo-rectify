import QtQuick 2.15
import QtQuick.Window 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.12
import Qt.labs.settings 1.0


Window {
    property var my_width: 1200
    width: 640
    height: 480
    visible: true
    title: qsTr("Hello World")

    Settings {
        category: "stereo_rectify"
        id: settings
        property string cam_l_f: "13.7"
        property string cam_r_f: "13.7"

        property int cam_l_proj: 0
        property int cam_r_proj: 0

        property int method: 0
    }

    ColumnLayout {
        width: parent.width
        RowLayout {
            width: parent.width
            Text {
                width: 12
                text: "Input focal length"
            }
            TextField {
                text: settings.cam_l_f
                onEditingFinished: {
                    cam_l.setFocal(text)
                    settings.cam_l_f = text
                }
                Component.onCompleted: cam_l.setFocal(text)
            }
            TextField {
                text: settings.cam_r_f
                onEditingFinished: {
                    cam_r.setFocal(text)
                    settings.cam_r_f = text
                }
                Component.onCompleted: cam_r.setFocal(text)
            }
        }
        RowLayout {
            Text {
                width: my_width
                text: "Input projection type"
            }
            ComboBox {
                width: 800
                model: [ "Rectilinear", "Equidistant" ]
                currentIndex: settings.cam_l_proj
                onCurrentValueChanged: {
                    cam_l.setProjection(currentText)
                    settings.cam_l_proj = currentIndex
                }
                Component.onCompleted: cam_l.setProjection(currentText)
            }
            ComboBox {
                width: 400
                model: [ "Rectilinear", "Equidistant" ]
                currentIndex: settings.cam_r_proj
                onCurrentValueChanged: {
                    cam_r.setProjection(currentText)
                    settings.cam_r_proj = currentIndex
                }
                Component.onCompleted: cam_r.setProjection(currentText)
            }
        }
        RowLayout {
            width: parent.width
            Text {
                width: 12
                text: "Optimization method"
            }
            ComboBox {
                width: 400
                model: [ "simple", "1-cam", "2-cam" ]
                currentIndex: settings.method
                onCurrentValueChanged: {
                    manager.setMethod(currentText)
                    settings.method = currentIndex
                }
                Component.onCompleted: manager.setMethod(currentText)
            }
            Button {
                text: "Run"
                onClicked: manager.run();
            }
        }
    }

}

