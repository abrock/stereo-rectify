# stereo-rectify
The goal of this project is to read two images and corresponding calibration data (e.g. from lensfun)
and automatically create a rectilinear rectified stereo pair suitable for creating an anaglyph image for viewing using red-cyan glasses.
The idea is to use feature matching like SIFT to automatically compute the relative rotation of the two cameras.
