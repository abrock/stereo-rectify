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

        property string scale_result: "1.0"
        property string target_focal: "8"
        property double rotate: 0.0

        property bool fix_left: false;
        property string fix_left_x: "0";
        property string fix_left_y: "0";
        property string fix_left_z: "0";
    }

    property var camsettings_label_width: 150
    property var camsettings_width: 120
    property var margin: 10

    ColumnLayout {
        ColumnLayout {
            Layout.margins: margin
            RowLayout {
                Text {
                    Layout.preferredWidth: camsettings_label_width
                    text: "Input focal length"
                }
                TextField {
                    Layout.preferredWidth: camsettings_width
                    text: settings.cam_l_f
                    onEditingFinished: {
                        cam_l.setFocal(text)
                        settings.cam_l_f = text
                    }
                    Component.onCompleted: cam_l.setFocal(text)
                }
                TextField {
                    Layout.preferredWidth: camsettings_width
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
                    Layout.preferredWidth: camsettings_label_width
                    text: "Input projection type"
                }
                ComboBox {
                    Layout.preferredWidth: camsettings_width
                    model: [ "Rectilinear", "Equidistant" ]
                    currentIndex: settings.cam_l_proj
                    onCurrentValueChanged: {
                        cam_l.setProjection(currentText)
                        settings.cam_l_proj = currentIndex
                    }
                    Component.onCompleted: cam_l.setProjection(currentText)
                }
                ComboBox {
                    Layout.preferredWidth: camsettings_width
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
                Button {
                    text: "Fix left cam orientation"
                    checkable: true
                    checked: settings.fix_left
                    id: fix_left
                    onClicked: {
                        settings.fix_left = checked
                        calib.setFixedLeftRot(fix_left.checked, fix_left_x.text, fix_left_y.text, fix_left_z.text)
                    }
                    Component.onCompleted: calib.setFixedLeftRot(fix_left.checked, fix_left_x.text, fix_left_y.text, fix_left_z.text)

                }
                Text {
                    text: "X:"
                }
                TextField {
                    Layout.preferredWidth: camsettings_width
                    text: settings.fix_left_x
                    id:fix_left_x
                    onEditingFinished: {
                        settings.fix_left_x = text
                        calib.setFixedLeftRot(fix_left.checked, fix_left_x.text, fix_left_y.text, fix_left_z.text)
                    }
                    Component.onCompleted: calib.setFixedLeftRot(fix_left.checked, fix_left_x.text, fix_left_y.text, fix_left_z.text)
                }
                Text {
                    text: "Y:"
                }
                TextField {
                    Layout.preferredWidth: camsettings_width
                    text: settings.fix_left_y
                    id:fix_left_y
                    onEditingFinished: {
                        settings.fix_left_y = text
                        calib.setFixedLeftRot(fix_left.checked, fix_left_x.text, fix_left_y.text, fix_left_z.text)
                    }
                    Component.onCompleted: calib.setFixedLeftRot(fix_left.checked, fix_left_x.text, fix_left_y.text, fix_left_z.text)
                }
                Text {
                    text: "Z:"
                }
                TextField {
                    Layout.preferredWidth: camsettings_width
                    text: settings.fix_left_z
                    id:fix_left_z
                    onEditingFinished: {
                        settings.fix_left_z = text
                        calib.setFixedLeftRot(fix_left.checked, fix_left_x.text, fix_left_y.text, fix_left_z.text)
                    }
                    Component.onCompleted: calib.setFixedLeftRot(fix_left.checked, fix_left_x.text, fix_left_y.text, fix_left_z.text)
                }

            }

            RowLayout {
                width: parent.width
                Text {
                    Layout.preferredWidth: camsettings_label_width
                    text: "Optimization method"
                }
                ComboBox {
                    Layout.preferredWidth: camsettings_width
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
                Text {
                    text: "Preview"
                }
                ComboBox {
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
                    text: "Clip limit"
                }
                TextField {
                    text: settings.ce_clip_limit
                    id: ce_clip_limit
                    Layout.preferredWidth: 60
                    onEditingFinished: {
                        settings.ce_clip_limit = text
                        manager.setCLAHE(enhance_contrast.checked, ce_clip_limit.text, ce_grid_size.text)
                    }
                    Component.onCompleted: manager.setCLAHE(enhance_contrast.checked, ce_clip_limit.text, ce_grid_size.text)
                }
                Text {
                    text: "Grid size"
                }
                TextField {
                    text: settings.ce_grid_size
                    id: ce_grid_size
                    Layout.preferredWidth: 60
                    onEditingFinished: {
                        settings.ce_grid_size = text
                        manager.setCLAHE(enhance_contrast.checked, ce_clip_limit.text, ce_grid_size.text)
                    }
                    Component.onCompleted: manager.setCLAHE(enhance_contrast.checked, ce_clip_limit.text, ce_grid_size.text)
                }
            }
            RowLayout {
                Text {
                    text: "Scale result: "
                }
                TextField {
                    text: settings.scale_result
                    id: scale_result
                    Layout.preferredWidth: 60
                    onEditingFinished: {
                        settings.scale_result = text
                        manager.setScaleResult(text)
                    }
                    Component.onCompleted: manager.setScaleResult(text)
                }
                Text {
                    text: "Target focal length: "
                }
                TextField {
                    text: settings.target_focal
                    id: target_focal
                    Layout.preferredWidth: 60
                    onEditingFinished: {
                        settings.target_focal = text
                        manager.setTargetFocal(text)
                    }
                    Component.onCompleted: manager.setTargetFocal(text)
                }
            }
            RowLayout {
                Text {
                    text: "Rotate result"
                }
                Slider {
                    from: -30
                    to: 30
                    value: settings.rotate
                    onMoved: {
                        settings.rotate = value
                        manager.setRotation(value)
                    }
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
                Button {
                    text: "Save"
                    onClicked: manager.save();
                }
            }
        }
    }

}

