# Compile Flags Bug: pgo 被静默编译成 `-O0`

## TL;DR

虽然顶层 CMakeLists 的 fallback 会把空的 `CMAKE_BUILD_TYPE` 置为 `Release`，且 `-O3 -DNDEBUG` 也确实出现在了编译命令里，但每个 pgo 目标（`src/core/*`、`src/tools/*`、`tests/*`）最终的 `CXX_FLAGS` **末尾都被追加了 `-O0`**。clang 取最后一个 `-O`，所以整个库实际跑在未优化 + `NDEBUG` 的模式下。这是 cubic/box 示例从 ~0.5s/iter 开始迅速劣化到 60+s/iter 的主因。

`base_no_mkl` preset 并不是一个"干净的 Release"：它完全不设 `CMAKE_BUILD_TYPE`，Release 属性只靠 `CMakeLists.txt:108-111` 的一次性 fallback 续命，而这条 fallback 被本 bug 里的第二条传播链完全覆盖了。

---

## 证据

### 1. 真正的编译命令

`build/base_no_mkl/src/core/contact/CMakeFiles/contact.dir/flags.make:9`:

```
CXX_FLAGS = -O3 -DNDEBUG -std=c++20 -arch arm64 -fPIC -Wall -Wextra \
            -frounding-math -fvisibility=hidden -march=native -mtune=native \
            -O3 -ggdb3 -O0
```

`-O0` 在最尾部，clang 按"最后一个 -O 胜出"取 `-O0`。与此同时 `-DNDEBUG` 被保留，所以断言是关闭的——「production-style 关 assert + debug-style 不优化」的最坏组合。

### 2. 污染面

在 `/tmp/pgo_cmake_dry` 做全新 configure，遍历 `src/core/*/flags.make`：

| 目标 | 是否含 `-O0` |
| --- | --- |
| `basicAlgorithms` | 否 |
| `basicIO` / `configFileJSON` / `constraintPotentialEnergies` / `contact` / `eigenSupport` / `genericPotentialEnergies` / `geometryPotentialEnergies` / `interpolationCoordinates` / `mesh` / `nonlinearOptimization` / `pgoLogging` / `simulation` / `solidDeformationModel` / `tetgenInterface` / `volumetricMesh` | 是 |

第三方 `tiny_obj_loader` / `CCD_SafeCCD` / `CCD_ExactCCD` / `stb` 未中招。

### 3. 根因——Eigen3 `INTERFACE_COMPILE_OPTIONS` 被永久污染

在 CMakeLists.txt 尾部临时插入 `get_target_property(...)` 做实测，`Eigen3::Eigen` 的 `INTERFACE_COMPILE_OPTIONS` 内容是（注意有两段）：

```
$<$<COMPILE_LANGUAGE:CXX>:-Wall> ; … ; -march=native ; -mtune=native ;
-O0 ; -ggdb3 ;                ← 第一段：从 debug 配置里烘进来
$<$<COMPILE_LANGUAGE:CXX>:-Wall> ; … ; -march=native ; -mtune=native ;
-O3 ; -ggdb3                   ← 第二段：当前 release 配置追加
```

而同一时刻 `compilation_flag` 的 `INTERFACE_COMPILE_OPTIONS` 是干净的：
```
… ; -O3 ; -ggdb3
```

也就是说污染 **只在 Eigen3 这个 target 上**。

落到磁盘的证据：
`build/base_no_mkl_debug/_deps/eigen3-build/Eigen3Targets.cmake`：

```cmake
set_target_properties(Eigen3::Eigen PROPERTIES
  INTERFACE_COMPILE_OPTIONS "...;-O0;-ggdb3;..."
  …
)
```

Debug 构建把 `-O0` 烘进了 Eigen3 的导出包。

### 4. 传播链

1. 早先某次 `base_no_mkl_debug` 配置把 Eigen3 下到 `build/base_no_mkl_debug/_deps/eigen3-src/`，并在 `_deps/eigen3-build/` 导出了 `Eigen3Config.cmake`/`Eigen3Targets.cmake`。在 Debug 模式下 `compilation_flag` 带 `-O0 -ggdb3`，被 `eigen.cmake` 注入到了 Eigen3 的 `INTERFACE_COMPILE_OPTIONS` 并写入了导出文件。此外 Eigen3 在 user package registry（`~/.cmake/packages/Eigen3/`）留下了索引条目指向这个 build 目录。

2. 现在配 `base_no_mkl`。`CMakeModules/third-party/eigen.cmake:10-18` 的 `FetchContent_Declare(... FIND_PACKAGE_ARGS NAMES Eigen3)` 让 CMake 先做 `find_package(Eigen3)`，命中 registry → 直接复用 debug build 那份 `Eigen3Config.cmake`。`-isystem .../base_no_mkl_debug/_deps/eigen3-src` 出现在 release build 的编译命令里，印证了这一点。

3. `eigen.cmake:39-41`:
   ```cmake
   get_target_property(flags compilation_flag INTERFACE_COMPILE_OPTIONS)
   message(STATUS "Eigen3 compilation flags: ${flags}")
   target_compile_options(${REAL_TGT} INTERFACE ${flags})
   ```
   用 `target_compile_options` **追加**而非覆盖。Eigen3 本来带着的 `-O0 -ggdb3` 没被清掉，现在又 append 上 `-O3 -ggdb3`。

4. `src/core/macros/CMakeLists.txt:4`：
   ```cmake
   target_link_libraries(macros INTERFACE compilation_flag)
   ```
   几乎所有 pgo 目标都通过 `macros`（或其它路径）间接链到 Eigen3（例如通过 `eigenSupport → Eigen3::Eigen`）。Eigen3 的 `INTERFACE_COMPILE_OPTIONS` 于是流进每个 pgo 目标的 `flags.make`。

5. CMake 在拼 `CXX_FLAGS` 时对相邻重复项有一定去重，所以 `-Wall/-Wextra/...` 只各出现一次；但 `-O0`/`-O3` 值不同，不会被合并。最终顺序让 `-O0` 落在 `-O3` 之后，clang 取 `-O0`。

### 5. `base_no_mkl` 并不是显式 Release

`build/base_no_mkl/CMakeCache.txt`：

```
CMAKE_BUILD_TYPE:STRING=
CMAKE_CXX_FLAGS:STRING=
CMAKE_CXX_FLAGS_RELEASE:STRING=-O3 -DNDEBUG
```

Release 行为靠 `CMakeLists.txt:108-111`：

```cmake
if(CMAKE_BUILD_TYPE STREQUAL "")
  message(STATUS "CMake build type is empty and will be set to Release")
  set(CMAKE_BUILD_TYPE Release)      # 只影响本作用域，不写回 CACHE
endif()
```

这条 fallback 让 `CMAKE_CXX_FLAGS_RELEASE=-O3 -DNDEBUG` 被用上，所以编译命令头部有 `-O3 -DNDEBUG`——是真的。但这是「勉强及格」的 Release，一旦有下游 target 追加 `-O0`（就是本 bug），fallback 挡不住。

### 6. 运行时观察到的变慢

cubic/box 日志（节选）：

| Iter | Newton solve | CD | # ext contacts |
| --- | --- | --- | --- |
| T465 | 0.54s | 0.20s | 516 |
| T480 | 1.86s | 0.23s | 1613 |
| T492 | 11.5s | 0.20s | 1483 |
| T494 | 32.6s | 1.01s | 1442 |
| T496 | 69.6s | 5.35s | 1391 |
| T501 | 68.6s | 3.88s | 389 |
| T506 | 29.6s | 2.27s | 0 |

接触数在掉但耗时继续涨——单靠 `-O0` 不能完全解释"越跑越慢"。推测是 `-O0` 把基础耗时放大 10–30×后，Newton/CD 分支里某些临时分配（`interpolationMatrix` 的 `tbb::concurrent_vector<TripletD>`、CD BVH rebuild 等）触发了 Mac 的内存压力/swap/thermal throttle。先修 `-O0` 再观察是否仍有渐进性衰减。

---

## 为什么这次特别容易触发

- `eigen.cmake:10-18` 使用 `FIND_PACKAGE_ARGS NAMES Eigen3`，FetchContent 优先走 find_package。用户 registry / 其他 build dir 的 `Eigen3Config.cmake` 都会被命中。
- `eigen.cmake:39-41` 是 **append** 语义。同一个 Eigen3 target 被不同 build type 反复 `target_compile_options`，内容会累积；且一旦被 Debug build 导出过，后续的 Release build import 它就把 `-O0` 带了进来。
- 顶层 `base_no_mkl` / `base` preset 没声明 `CMAKE_BUILD_TYPE`，导致第一条不合理的路径：Release 依赖 CMakeLists fallback 而不是 preset 保证。

---

## 建议修复（按稳定性从高到低）

### A. 修 `CMakeModules/third-party/eigen.cmake`（必须）

核心问题：不应该在 Eigen3 这个可导出的 imported target 上做 append。两种选项任选其一。

**选项 A1（兼容性保底）**：改成覆盖写，每次 configure 都用当前 `compilation_flag` 的 flags 覆盖 Eigen3 的 `INTERFACE_COMPILE_OPTIONS`，防止累积。

```cmake
get_target_property(flags compilation_flag INTERFACE_COMPILE_OPTIONS)
message(STATUS "Eigen3 compilation flags: ${flags}")
set_property(TARGET ${REAL_TGT} PROPERTY INTERFACE_COMPILE_OPTIONS ${flags})
```

**选项 A2（当前仓库首选）**：直接删掉这几行。消费者已经通过 `target_link_libraries(... compilation_flag)`（经由 `add_libpgo_lib` 或 `macros`）拿到 `compilation_flag` 的所有 flag，Eigen3 不需要再二次传递。只有纯粹只链 Eigen3 但不链 compilation_flag 的 target 会丢到 flag——在本仓库里没有这种 target。

之所以优先 A2，是因为它更符合职责边界：`Eigen3::Eigen` 只代表 Eigen 自己，`compilation_flag` 只代表 libpgo 自己的编译策略，不再把项目策略“写回”第三方 target。这样比 A1 更不容易再次触发导出污染、跨 build type 复用污染、user package registry 串台等问题。

### B. 修 `CMakePresets.json`（必须）

`base`（line 22-32）、`base_no_mkl`（line 10-20）、`base_win`（line 34-44）显式加 `"CMAKE_BUILD_TYPE": "Release"`，不要靠 CMakeLists 的 fallback。验证：

- `all_release` / `base_cuda_release` 已经自己设 Release，加了也是覆盖成 Release 自身，无副作用。
- `all_debug` / `base_cuda_debug` / `base_no_mkl_debug` inherit 顺序是 `base*, ..., debug`，`debug` 在后，仍然拿到 Debug。

### C. 清污染（一次性、必要）

只改代码不够——本机上的 registry/build dir 还在污染。执行：

```bash
cd /Users/jinceyang/Desktop/codebase/libpgo
rm -rf build/base_no_mkl build/base_no_mkl_debug
rm -rf ~/.cmake/packages/Eigen3   # 清 user package registry
cmake --preset base_no_mkl
cmake --build build/base_no_mkl -j
```

验证命令：

```bash
grep "^CXX_FLAGS" build/base_no_mkl/src/core/contact/CMakeFiles/contact.dir/flags.make
# 期望尾巴是 -O3 -ggdb3，不再出现 -O0
```

### D. 长期加固（可选，建议）

- 顶层 `CMakeLists.txt:108-111` 的 fallback 改成写回 CACHE，避免在 preset 没设 build type 时各处行为不一致：
  ```cmake
  if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
  endif()
  ```
- `eigen.cmake` 的 `FetchContent_Declare` 去掉 `FIND_PACKAGE_ARGS NAMES Eigen3`（或改为只在指定 root 下搜索），避免被其他 build dir 的导出包"串台"。

---

## 这类问题的最佳实践

核心原则：**不要把“本项目自己的编译策略”直接写到第三方 target 身上，尤其是可导出、可复用、可能来自 `find_package()` 的 target。**

更稳的做法：

- 把项目级 flag 留在自己的 `INTERFACE` target 上（本仓库就是 `compilation_flag` / `compilation_flag_for_debug`）。
- 业务 target 直接 `target_link_libraries(... compilation_flag)`，不要通过 `Eigen3::Eigen`、`TBB::tbb` 之类第三方 target 做二次转发。
- 如果确实需要给某个第三方库加“本项目专用配置”，优先包一层本地 wrapper target，例如 `pgo_eigen`，由 wrapper 去链接 `Eigen3::Eigen` 和 `compilation_flag`，而不是改 `Eigen3::Eigen` 本体。
- 对 `FetchContent`，要区分“允许复用系统/用户环境里的包”和“必须使用仓库 pin 住的版本”。如果追求可复现，避免默认走 user package registry；可以去掉 `FIND_PACKAGE_ARGS`、显式传 `NO_DEFAULT_PATH`，或设置 `CMAKE_FIND_USE_PACKAGE_REGISTRY=FALSE`。
- `CMAKE_BUILD_TYPE` 应该在 preset 里显式写明，不要依赖顶层 `CMakeLists.txt` 的 fallback 补救。

一个更干净的 Eigen 组织方式大致是：

```cmake
add_library(pgo_eigen INTERFACE)
target_link_libraries(pgo_eigen INTERFACE Eigen3::Eigen compilation_flag)

if(TARGET MKL::MKL)
  target_link_libraries(pgo_eigen INTERFACE MKL::MKL)
  target_compile_definitions(pgo_eigen INTERFACE
    EIGEN_DONT_PARALLELIZE
    EIGEN_USE_MKL_ALL
    EIGEN_MKL_NO_DIRECT_CALL)
endif()

target_compile_definitions(pgo_eigen INTERFACE EIGEN_MAX_ALIGN_BYTES=32)
```

这样项目策略留在本地 wrapper 上，第三方 target 本体保持“干净”，即使被导出、复用、跨 build type import，也不容易被污染。

---

## 仓库内类似风险扫描

### 已确认的同级别问题

目前明确命中的就是 `Eigen3` 这一处：

- `CMakeModules/third-party/eigen.cmake:10-18` 同时使用了 `FetchContent_Declare(... FIND_PACKAGE_ARGS NAMES Eigen3)` 和 `FetchContent_MakeAvailable(Eigen3)`，会优先走 `find_package(Eigen3)`，因此会命中 user package registry。
- `CMakeModules/third-party/eigen.cmake:33-42` 又直接修改了 `Eigen3::Eigen`（或其真实 target）的 `INTERFACE_*` 属性，其中 `target_compile_options(... INTERFACE ${flags})` 还使用了 append 语义。

这两点叠加，才导致了“debug build 导出的旧 flags 被 release build 复用并继续累积”的问题。

### 结构相似、但暂未发现同样污染结果的地方

仓库里还有不少 third-party 模块使用了 `FetchContent_Declare(... FIND_PACKAGE_ARGS ...)` + `FetchContent_MakeAvailable(...)`：

- `CMakeModules/third-party/argparse.cmake`
- `CMakeModules/third-party/arpackng.cmake`
- `CMakeModules/third-party/autodiff.cmake`
- `CMakeModules/third-party/ceres.cmake`
- `CMakeModules/third-party/cmaes.cmake`
- `CMakeModules/third-party/cuCollections.cmake`
- `CMakeModules/third-party/fmt.cmake`
- `CMakeModules/third-party/geogram.cmake`
- `CMakeModules/third-party/glfw.cmake`
- `CMakeModules/third-party/glm.cmake`
- `CMakeModules/third-party/nlohmann_json.cmake`
- `CMakeModules/third-party/nlopt.cmake`
- `CMakeModules/third-party/pybind11.cmake`
- `CMakeModules/third-party/spdlog.cmake`
- `CMakeModules/third-party/suitesparse.cmake`

这些模块目前**没有**像 Eigen 一样再去追加本项目 compile flags 到外部 target 上，所以还没看到同级别的 `-O0` 污染；但它们仍然存在“依赖来源可能受本机环境 / user package registry 影响”的可复现性风险。

### 风险较低但值得记录的外部 target 修改

- `CMakeModules/third-party/tbb.cmake:76-83` 会在缺少 `IMPORTED_LOCATION_RELEASE` 时改写 `TBB::tbb` 的 imported location，把 `RELWITHDEBINFO` 挪作 `RELEASE` 兜底。

这不属于本次 compile flags 污染问题，但仍然是“修改外部 target 本体”的模式。长期看，如果后续还要加项目专用设置，最好也改成 wrapper target，而不是继续往 `TBB::tbb` 本体上堆属性。

### 相对更安全的例子

- `CMakeModules/third-party/cgal.cmake:62` 最终使用的是：
  ```cmake
  find_package(CGAL CONFIG COMPONENTS Core REQUIRED PATHS ${CGAL_SOURCE_DIR} NO_DEFAULT_PATH)
  ```
  这会把查找路径钉死在当前 fetch 出来的源码目录，不会去命中 user package registry。即使它前面 `FetchContent_Declare` 里保留了 `FIND_PACKAGE_ARGS`，后续显式的 `find_package(... NO_DEFAULT_PATH)` 仍把来源收回到了本地。

### 与本 bug 强相关的另一处脆弱点

- `CMakePresets.json` 里的 `base` / `base_no_mkl` / `base_win` 没有显式设置 `CMAKE_BUILD_TYPE`。

这不是“污染进入 registry”的直接原因，但属于同一种“依赖隐式状态”的设计问题。只要 build type 不在 preset 中固定，就更容易让不同 configure 路径、不同缓存状态之间互相污染或表现不一致。

---

## 复现和验证

### 复现

```bash
cd /Users/jinceyang/Desktop/codebase/libpgo
cmake --preset base_no_mkl_debug           # 先配 debug，污染 Eigen3
cmake --build build/base_no_mkl_debug -t eigenSupport   # 让它完成一次 generate
cmake --preset base_no_mkl                 # 再配 release
grep "^CXX_FLAGS" build/base_no_mkl/src/core/contact/CMakeFiles/contact.dir/flags.make
# 观察尾部 -O0
```

### 验证 A 修复生效

在应用 A1 修复后，在 `CMakeLists.txt` 末尾临时加：

```cmake
get_target_property(opts Eigen3::Eigen INTERFACE_COMPILE_OPTIONS)
message(STATUS "CHECK Eigen3 opts: ${opts}")
```

重新 configure `base_no_mkl`。期望：输出里只出现一段 flags，且含 `-O3` 不含 `-O0`。

### 验证 C 清污染生效

`flags.make` 尾部不再有 `-O0`。进一步跑 cubic/box，观察 Newton solve 时间应回到 < 100ms 量级。

---

## 相关文件

- `CMakeLists.txt:82-201` — `compilation_flag` / `compilation_flag_for_debug` 的定义与 Release/Debug 分支
- `CMakeLists.txt:108-111` — build type fallback
- `CMakePresets.json:10-44` — `base` / `base_no_mkl` / `base_win` 缺 `CMAKE_BUILD_TYPE`
- `CMakeModules/third-party/eigen.cmake:10-18` — 带 `FIND_PACKAGE_ARGS` 的 FetchContent
- `CMakeModules/third-party/eigen.cmake:39-41` — 把 `compilation_flag` flags append 到 Eigen3 的 `INTERFACE_COMPILE_OPTIONS`
- `CMakeModules/CompilationUtils.cmake:1-61` — `add_flag` / `add_flag_poxis` 实现
- `src/core/macros/CMakeLists.txt:4` — 传播 `compilation_flag` 的枢纽
- `build/base_no_mkl_debug/_deps/eigen3-build/Eigen3Targets.cmake` — 被 debug build 烘进 `-O0` 的导出文件
