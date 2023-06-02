#ifndef STEREOMANAGER_H
#define STEREOMANAGER_H

#include <QObject>

#include "cam.h"
#include "calib.h"

class StereoManager : public QObject {
    Q_OBJECT
public:

    std::shared_ptr<Cam> cam_l;
    std::shared_ptr<Cam> cam_r;
    std::shared_ptr<Cam> cam_target_l;
    std::shared_ptr<Cam> cam_target_r;
    std::shared_ptr<Calib> calib;

    enum class Method {simple, cam1, cam2};

    Method method = Method::simple;

    bool contrast_enhanced = false;

    double baseline = 10;

    Method str2method(std::string str);

    Q_INVOKABLE void setMethod(QString const& str);

    Q_INVOKABLE void run();

    void optimize();
};

#endif // STEREOMANAGER_H