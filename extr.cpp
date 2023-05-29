#include "extr.h"

Extr::Extr()
{

}

void Extr::setConstant(ceres::Problem &problem) {
    problem.SetParameterBlockConstant(rot.val);
    problem.SetParameterBlockConstant(loc.val);
}
