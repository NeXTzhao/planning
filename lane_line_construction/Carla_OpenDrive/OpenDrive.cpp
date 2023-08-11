// Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "OpenDrive.h"
#include <boost/filesystem/operations.hpp>
#include <fstream>

namespace fs = boost::filesystem;

bool Match(char const *str, char const *test) {
    return 0 == fnmatch(test, str, 0);
}

void ValidateFilePath(std::string &filepath, std::string const &ext) {
    fs::path path(filepath);
    if (path.extension().empty() && !ext.empty()) {
        if (ext[0] != '.') {
            path += '.';
        }
        path += ext;
    }
    auto parent = path.parent_path();
    if (!parent.empty()) {
        fs::create_directories(parent);
    }
    filepath = path.string();
}

std::vector<std::string> ListFolder(std::string const &folder_path,
                                    std::string const &wildcard_pattern) {
    fs::path root(folder_path);
    if (!fs::exists(root) || !fs::is_directory(root)) {
        std::cout << "[error]" << std::endl;
    }

    std::vector<std::string> results;
    fs::directory_iterator end;
    for (fs::directory_iterator it(root); it != end; ++it) {
        if (fs::is_regular_file(*it)) {
            const std::string filename = it->path().filename().string();
            if (Match(filename.c_str(), wildcard_pattern.c_str())) {
                results.emplace_back(filename);
            }
        }
    }
    return results;
}

namespace util {

    std::vector<std::string> OpenDrive::GetAvailableFiles() {
        return ListFolder(LIBOPENDRIVE_TEST_CONTENT_FOLDER "/OpenDrive/", "*.xodr");
    }

    std::string OpenDrive::Load(const std::string &filename) {
        const std::string opendrive_folder =
                LIBOPENDRIVE_TEST_CONTENT_FOLDER "/OpenDrive/";
        std::ifstream file(opendrive_folder + filename);
        return std::string{std::istreambuf_iterator<char>(file),
                           std::istreambuf_iterator<char>()};
    }

}  // namespace util
