import QtQuick 2.15
import QtQuick.Window 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.12


Window {
    property var my_width: 1200
    width: 640
    height: 480
    visible: true
    title: qsTr("Hello World")

    ColumnLayout {
        width: parent.width
        RowLayout {
            width: parent.width
            Text {
                width: 12
                text: "Input focal length"
            }
            TextField {
                text: "24"
                onEditingFinished: cam_l.setFocal(text);
                Component.onCompleted: cam_l.setFocal(text);
            }
            TextField {
                text: "24"
                onEditingFinished: cam_r.setFocal(text);
                Component.onCompleted: cam_r.setFocal(text);
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
                onCurrentIndexChanged: cam_l.setProjection(currentText)
                Component.onCompleted: cam_l.setProjection(currentText)
            }
            ComboBox {
                width: 400
                model: [ "Rectilinear", "Equidistant" ]
                onCurrentIndexChanged: cam_r.setProjection(currentText)
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
                onCurrentIndexChanged: manager.setMethod(currentText)
                Component.onCompleted: manager.setMethod(currentText)
            }
            Button {
                text: "Run"
                onClicked: manager.run();
            }
        }
    }

}

