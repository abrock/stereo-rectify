#include <iostream>

#include <filesystem>
namespace fs = std::filesystem;

#include "misc.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <libraw/libraw.h>

using namespace Misc;

class StereoSortManager {

    StereoSortManager() {}


public:
    static StereoSortManager& getInstance()
    {
        static StereoSortManager    instance; // Guaranteed to be destroyed.
        // Instantiated on first use.
        return instance;
    }

    StereoSortManager(StereoSortManager const&) = delete;
    void operator=(StereoSortManager const&)    = delete;

    fs::path input_dir_l;
    fs::path input_dir_r;
    fs::path output_dir;

    std::string name_l = "left";
    std::string name_r = "right";

    size_t index_l = 0;
    size_t index_r = 0;

    std::vector<fs::path> files_l;
    std::vector<fs::path> files_r;

    cv::Mat read_raw_thumb(fs::path const& path) {
        cv::Mat result;
        LibRaw raw;
        int const code = raw.open_file(path.string().c_str());

        if (0 != code) {
            return result;
        }
        // Let us unpack the thumbnail
        if(int ret = raw.unpack_thumb() != LIBRAW_SUCCESS)
        {
            // error processing is completely similar to the previous case
            Misc::println("Cannot unpack_thumb {}: {}", path.string(), libraw_strerror(ret));
            if(LIBRAW_FATAL_ERROR(ret))
                return result;
        }
        else // We have successfully unpacked the thumbnail, now let us write it to a file
        {
            std::string const thumb_fn = LIBRAW_THUMBNAIL_JPEG ? "/tmp/stereo-sort-thumb.jpg" : "/tmp/stereo-sort-thumb.ppm";
            if( LIBRAW_SUCCESS != (ret = raw.dcraw_thumb_writer(thumb_fn.c_str())))
            {
                Misc::println("Cannot write {}: {}", thumb_fn, libraw_strerror(ret));
                // in the case of fatal error, we should terminate processing of the current file
                if(LIBRAW_FATAL_ERROR(ret))
                    return result;
            }
            return cv::imread(thumb_fn);
        }
        return result;
    }

    bool is_raw(fs::path const& path) {
        return !read_raw_thumb(path).empty();
    }

    bool is_readable_by_cv(fs::path const& path) {
        cv::Mat img = cv::imread(path, cv::IMREAD_GRAYSCALE);
        return !img.empty();
    }

    bool is_img(fs::path const& path) {
        return is_readable_by_cv(path) || is_raw(path);
    }

    cv::Mat read_img_or_thumb(fs::path const& path) {
        cv::Mat result = cv::imread(path.string());
        if (!result.empty()) {
            return result;
        }
        return read_raw_thumb(path);
    }

    void list_imgs(fs::path const& dir, std::vector<fs::path>& vec) {
        for (auto it = fs::recursive_directory_iterator(dir); it != fs::recursive_directory_iterator(); ++it) {
            if (is_img(it->path())) {
                vec.push_back(it->path());
            }
        }
        std::sort(vec.begin(), vec.end());
    }

    void print_vec(std::vector<fs::path> const& vec) {
        for (size_t ii = 0; ii < vec.size(); ++ii) {
            Misc::println("#{:>3}: {}", ii, vec[ii].string());
        }
    }

    void mouse_call_back(int event, int x, int y, int flags, std::string const& window) {
        Misc::println("Mouse event {}, pt ({}, {}), flags {}, window {}", event, x, y, flags, window);
    }

    static void CallBackFunc(int event, int x, int y, int flags, void* userdata) {
        getInstance().mouse_call_back(event, x, y, flags, *static_cast<std::string*>(userdata));
    }

    static std::string exec(const char* cmd) {
        std::array<char, 128> buffer;
        std::string result;
        std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
        if (!pipe) {
            throw std::runtime_error("popen() failed!");
        }
        while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
            result += buffer.data();
        }
        return result;
    }

    std::string active_window_name() {
        return exec("xdotool getactivewindow getwindowname");
    }

    void init() {
        cv::namedWindow(name_l);
        cv::namedWindow(name_r);

        //cv::setMouseCallback(name_l, &CallBackFunc, &name_l);
        //cv::setMouseCallback(name_r, &CallBackFunc, &name_r);

        list_imgs(input_dir_l, files_l);
        list_imgs(input_dir_r, files_r);

        Misc::println("#{}: {}, #{}: {}", name_l, files_l.size(), name_r, files_r.size());
        Misc::println("\nAll {} images: ", name_l);
        print_vec(files_l);
        Misc::println("\nAll {} images: ", name_r);
        print_vec(files_r);
    }
    std::string active_window;
    bool focus_left = true;

    void update_index(int const step) {
        size_t& val = focus_left ? index_l : index_r;
        size_t const max = focus_left ? files_l.size() : files_r.size();
        if (0 == max) {
            val = 0;
            return;
        }
        int new_val = int(val) + step;
        new_val = std::max(0, new_val);
        new_val = std::min(int(max)-1, new_val);
        val = new_val;
    }

    void move_files() {
        fs::path src_l = files_l[index_l];
        fs::path src_r = files_r[index_r];
        std::string basename_l = src_l.stem().string();
        std::string basename_r = src_r.stem().string();

        std::string const tgt_dir = basename_l + "_" + basename_r;
        std::string tgt_l = tgt_dir + "_" + name_l + src_l.extension().string();
        std::string tgt_r = tgt_dir + "_" + name_r + src_r.extension().string();

        println("Moving {} -> {}", src_l.string(), tgt_l);
        println("Moving {} -> {}", src_r.string(), tgt_r);

        fs::path const tgt_path_l = output_dir / tgt_l;
        fs::path const tgt_path_r = output_dir / tgt_r;

        if (fs::exists(tgt_path_l) || fs::exists(tgt_path_r)) {
            if (fs::exists(tgt_path_l)) {
                println("Target {} already exists, doing nothing.", tgt_path_l.string());
            }
            if (fs::exists(tgt_path_r)) {
                println("Target {} already exists, doing nothing.", tgt_path_r.string());
            }
            return;
        }
        if (tgt_path_l == tgt_path_r) {
            throw std::runtime_error("Target paths for left and right image are identical: " + tgt_path_r.string());
        }

        fs::create_directories(output_dir);

        fs::rename(src_l, tgt_l);
        fs::rename(src_r, tgt_r);
    }

    void run() {
        cv::imshow(name_l, read_img_or_thumb(files_l[index_l]));
        cv::imshow(name_r, read_img_or_thumb(files_r[index_r]));

        char const key = cv::waitKey(0);
        active_window = active_window_name();
        Misc::trim(active_window);
        focus_left = active_window == name_l;
        Misc::println("Active window: {}", active_window);
        Misc::println("focus_left: {}", focus_left);

        switch (key) {
        case 'q': exit(0);
        case 'w': update_index(1); break;
        case 's': update_index(-1); break;
        case 'd': move_files(); break;
        }
    }
};


int main(int argc, char ** argv) {
    if (argc < 4) {
        Misc::println("Usage: {} <input directory with left images> <input directory with right images> <output directory>", argv[0]);
        return EXIT_FAILURE;
    }

    StereoSortManager& mngr = StereoSortManager::getInstance();
    mngr.input_dir_l = argv[1];
    mngr.input_dir_r = argv[2];
    mngr.output_dir = argv[3];

    mngr.init();

    while (true) {
        mngr.run();
    }

}
