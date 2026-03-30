#pragma once

#include <string>

namespace pgo::FileService {

std::string normalizePath(const std::string& path);
std::string makeAbsolutePath(const std::string& path);
std::string getPathParentDirectory(const std::string& path);
std::string resolvePathRelativeToDirectory(const std::string& directoryPath, const std::string& path);

struct ResolvedConfigPath {
    std::string filePath;
    std::string directoryPath;

    std::string resolve(const std::string& path) const;
};

class ConfigPathResolver {
public:
    explicit ConfigPathResolver(const std::string& configFilePath);

    const ResolvedConfigPath& resolvedConfigPath() const { return resolvedConfigPath_; }
    const std::string&        filePath() const { return resolvedConfigPath_.filePath; }
    const std::string&        directoryPath() const { return resolvedConfigPath_.directoryPath; }
    std::string               resolve(const std::string& path) const { return resolvedConfigPath_.resolve(path); }

private:
    ResolvedConfigPath resolvedConfigPath_;
};

}  // namespace pgo::FileService
