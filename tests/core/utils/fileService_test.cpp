#include "fileService.h"

#include <gtest/gtest.h>

#include <filesystem>
#include <string>

namespace test {

namespace {
std::string parentOrDot(const std::string& path) {
    const std::string parent = std::filesystem::path(path).parent_path().lexically_normal().string();
    return parent.empty() ? std::string(".") : parent;
}
}  // namespace

TEST(FileServiceTest, NormalizePathHandlesEmptyAndDotSegments) {
    EXPECT_TRUE(pgo::FileService::normalizePath("").empty());

    const std::string input    = "a/./b/../c//d";
    const std::string expected = std::filesystem::path(input).lexically_normal().string();
    EXPECT_EQ(pgo::FileService::normalizePath(input), expected);
}

TEST(FileServiceTest, MakeAbsolutePathMatchesFilesystemAbsolute) {
    EXPECT_TRUE(pgo::FileService::makeAbsolutePath("").empty());

    const std::string relativePath = "tests/../examples/box/box.json";
    const std::string expected = std::filesystem::absolute(std::filesystem::path(relativePath)).lexically_normal().string();

    EXPECT_EQ(pgo::FileService::makeAbsolutePath(relativePath), expected);
}

TEST(FileServiceTest, ParentDirectoryFallbackToDot) {
    EXPECT_EQ(pgo::FileService::getPathParentDirectory(""), ".");
    EXPECT_EQ(pgo::FileService::getPathParentDirectory("single.txt"), ".");
    EXPECT_EQ(pgo::FileService::getPathParentDirectory("folder/sub/file.txt"),
              std::filesystem::path("folder/sub/file.txt").parent_path().lexically_normal().string());
}

TEST(FileServiceTest, ResolvePathRelativeToDirectoryHandlesPathKinds) {
    const std::string directory = "configs/run";

    const std::string relativePath = "../assets/mesh.obj";
    const std::string expectedRelative =
        (std::filesystem::path(directory) / std::filesystem::path(relativePath)).lexically_normal().string();
    EXPECT_EQ(pgo::FileService::resolvePathRelativeToDirectory(directory, relativePath), expectedRelative);

    const std::string absolutePath = std::filesystem::path("/tmp/../var/data.obj").string();
    const std::string expectedAbsolute = std::filesystem::path(absolutePath).lexically_normal().string();
    EXPECT_EQ(pgo::FileService::resolvePathRelativeToDirectory(directory, absolutePath), expectedAbsolute);

    EXPECT_TRUE(pgo::FileService::resolvePathRelativeToDirectory(directory, "").empty());
}

TEST(FileServiceTest, ConfigPathResolverBuildsAbsoluteFileAndDirectory) {
    const std::string configPath = "tmp/../tmp/config/sim.json";

    const pgo::FileService::ConfigPathResolver resolver(configPath);

    const std::string expectedFile = std::filesystem::absolute(std::filesystem::path(configPath)).lexically_normal().string();
    const std::string expectedDir  = parentOrDot(expectedFile);
    EXPECT_EQ(resolver.filePath(), expectedFile);
    EXPECT_EQ(resolver.directoryPath(), expectedDir);

    const std::string includePath      = "../assets/fixed.txt";
    const std::string expectedResolved =
        (std::filesystem::path(expectedDir) / std::filesystem::path(includePath)).lexically_normal().string();
    EXPECT_EQ(resolver.resolve(includePath), expectedResolved);
    EXPECT_TRUE(resolver.resolve("").empty());

    const pgo::FileService::ResolvedConfigPath resolvedConfigPath{resolver.filePath(), resolver.directoryPath()};
    EXPECT_EQ(resolvedConfigPath.resolve(includePath), expectedResolved);
}

}  // namespace test