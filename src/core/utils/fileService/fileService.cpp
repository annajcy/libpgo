#include "fileService.h"

#include <filesystem>

namespace pgo::FileService {

std::string normalizePath(const std::string& path) {
    if (path.empty()) {
        return path;
    }

    return std::filesystem::path(path).lexically_normal().string();
}

std::string makeAbsolutePath(const std::string& path) {
    if (path.empty()) {
        return path;
    }

    std::filesystem::path normalizedPath(path);
    if (!normalizedPath.is_absolute()) {
        normalizedPath = std::filesystem::absolute(normalizedPath);
    }

    return normalizedPath.lexically_normal().string();
}

std::string getPathParentDirectory(const std::string& path) {
    if (path.empty()) {
        return ".";
    }

    const std::string parentPath = std::filesystem::path(normalizePath(path)).parent_path().lexically_normal().string();
    return parentPath.empty() ? std::string(".") : parentPath;
}

std::string resolvePathRelativeToDirectory(const std::string& directoryPath, const std::string& path) {
    if (path.empty()) {
        return path;
    }

    std::filesystem::path resolvedPath(path);
    if (!resolvedPath.is_absolute()) {
        resolvedPath = std::filesystem::path(directoryPath) / resolvedPath;
    }

    return resolvedPath.lexically_normal().string();
}

std::string ResolvedConfigPath::resolve(const std::string& path) const {
    return resolvePathRelativeToDirectory(directoryPath, path);
}

ConfigPathResolver::ConfigPathResolver(const std::string& configFilePath) {
    resolvedConfigPath_.filePath      = makeAbsolutePath(configFilePath);
    resolvedConfigPath_.directoryPath = getPathParentDirectory(resolvedConfigPath_.filePath);
}

}  // namespace pgo::FileService
