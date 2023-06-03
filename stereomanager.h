#ifndef STEREOMANAGER_H
#define STEREOMANAGER_H

#include <QObject>

#include "cam.h"
#include "calib.h"

#include <ParallelTime/paralleltime.h>

class Cam;
class Calib;

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

    std::string preview = "red-cyan";
    Q_INVOKABLE void setPreview(QString const& str);

    bool use_clahe = false;
    double clahe_clip_limit = 4.0;
    int clahe_grid_size = 8;
    Q_INVOKABLE void setCLAHE(bool const checked, QString const& _clip_limit, QString const& _grid_size);

    Q_INVOKABLE void run();

    /**
     * @brief setAutoRun enables / disables running the preview automatically every time a setting is changed.
     * @param val
     */
    Q_INVOKABLE void setAutoRun(bool const val);

    /**
     * @brief autoRun calls run() if auto_run is true.
     */
    void autoRun();

    void optimize();

private:
    bool auto_run = false;
};

#endif // STEREOMANAGER_H
