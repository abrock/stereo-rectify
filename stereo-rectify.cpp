#include <iostream>

#include <tclap/CmdLine.h>

#include <opencv2/highgui.hpp>

#include "cam.h"

int main(int argc, char ** argv) {
    TCLAP::CmdLine cmd("stereo rectification tool");

    TCLAP::ValueArg<std::string> left_img_arg("l", "left", "left input image", true, "", "left input image", cmd);
    TCLAP::ValueArg<std::string> right_img_arg("r", "right", "right input image", true, "", "right input image", cmd);

    cmd.parse(argc, argv);



    return EXIT_SUCCESS;
}
