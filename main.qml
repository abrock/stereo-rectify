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
    title: qsTr("Stereo rectification tool")

    Settings {
        category: "stereo_rectify"
        id: settings
        property string cam_l_f: "13.7"
        property string cam_r_f: "13.7"

        property int cam_l_proj: 0
        property int cam_r_proj: 0

        property int method: 0

        property int preview_type: 0

        property bool enhance_contrast: false
        property string ce_clip_limit: "4.0"
        property string ce_grid_size: "8"
    }

    ColumnLayout {
        width: parent.width
        RowLayout {
            width: parent.width
            Text {
                width: 120
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
        }
        RowLayout {
            width: parent.width
            Text {
                width: 12
                text: "Preview"
            }
            ComboBox {
                width: 400
                model: [ "red-cyan", "left", "right", "side-by-side" ]
                currentIndex: settings.preview_type
                onCurrentValueChanged: {
                    manager.setPreview(currentText)
                    settings.preview_type = currentIndex
                }
                Component.onCompleted: manager.setPreview(currentText)
            }
        }
        RowLayout {
            width: parent.width
            Button {
                text: "Enhance contrast (CLAHE)"
                checkable: true
                checked: settings.enhance_contrast
                id: enhance_contrast
                onClicked: {
                    settings.enhance_contrast = checked
                    manager.setCLAHE(enhance_contrast.checked, ce_clip_limit.text, ce_grid_size.text)
                }
                Component.onCompleted: manager.setCLAHE(enhance_contrast.checked, ce_clip_limit.text, ce_grid_size.text)
            }
            Text {
                width: 120
                text: "Clip limit"
            }
            TextField {
                text: settings.ce_clip_limit
                id: ce_clip_limit
                onEditingFinished: {
                    settings.ce_clip_limit = text
                    manager.setCLAHE(enhance_contrast.checked, ce_clip_limit.text, ce_grid_size.text)
                }
                Component.onCompleted: manager.setCLAHE(enhance_contrast.checked, ce_clip_limit.text, ce_grid_size.text)
            }
            Text {
                width: 120
                text: "Grid size"
            }
            TextField {
                text: settings.ce_grid_size
                id: ce_grid_size
                onEditingFinished: {
                    settings.ce_grid_size = text
                    manager.setCLAHE(enhance_contrast.checked, ce_clip_limit.text, ce_grid_size.text)
                }
                Component.onCompleted: manager.setCLAHE(enhance_contrast.checked, ce_clip_limit.text, ce_grid_size.text)
            }
        }
        RowLayout {
            Button {
                text: "Run"
                onClicked: manager.run();
            }
            Button {
                text: "Autorun after every settings change"
                checkable: true
                onClicked: manager.setAutoRun(checked);
            }
        }
    }

}

