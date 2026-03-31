# volumetricMesh Phase C 执行文档

## 1. 结论与目标

基于当前代码状态，Phase B 可以视为已经完成，理由有三条：

1. 几何与材料存储已经从 `VolumetricMesh` 主体拆出，改为 `m_geometry` 与 `m_material_catalog`
2. `editing` 已经改成通过 `internal::MeshSubsetEditor` 工作，不再直接在 `volumetricMeshEdit.cpp` 里穿透字段
3. Phase B 文档里要求的最小验证集已经通过：
   - `core.scene.volumetric_mesh`
   - `core.energy.scene_to_simulation_mesh`
   - `core.energy.material_binding`
   - `api.pgo_c.basic`
   - `cubicMesher.tool.uniform`

因此 Phase C 可以正式开始。

Phase C 的目标很明确：**拆掉 `volumetricMeshIO.cpp` 这个 1000+ 行单体，并把 I/O 从“一个大文件 + 一个暴露 detail 的 header”改成“薄 façade + 明确的 reader / writer / detector / serde 边界”。**

## 2. 当前基线

### 2.1 已确认的现状

- [volumetricMeshIO.cpp](/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/volumetricMeshIO.cpp) 仍有 `1076` 行
- [volumetricMeshIO.h](/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/volumetricMeshIO.h) 仍把 `io::detail::LoadedMeshData` 和 `load_*_data()` 暴露在 public header 中
- [volumetricMeshParser.h](/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/volumetricMeshParser.h) 仍是模块根目录下的一等头文件
- [tetMesh.cpp](/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/tetMesh.cpp) 与 [cubicMesh.cpp](/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/cubicMesh.cpp) 仍直接依赖 `io::detail::load_*_data()`
- API / tools / tests 仍直接依赖：
  - `io::save`
  - `io::save_to_ascii`
  - `io::save_to_binary`
  - `TetMesh(path)` / `CubicMesh(path)` 文件构造路径

### 2.2 Phase C 需要解决的核心问题

当前 I/O 代码仍把下面几类职责压在一起：

- ASCII 读取
- Binary 读取
- ASCII 写出
- Binary 写出
- 文件格式探测
- 元素类型探测
- 材料反序列化 / 序列化
- `VolumetricMeshParser` 文本解析
- `LoadedMeshData` 中间态定义

Phase B 已经把“内部数据结构”理顺，Phase C 就不该再让 I/O 继续直接摊平处理原始数组和材料细节。

## 3. Phase C 范围与非目标

### 3.1 本阶段范围

- 拆分 `volumetricMeshIO.cpp`
- 收回 `volumetricMeshIO.h` 里暴露的 detail 类型和 detail 函数
- 把 `VolumetricMeshParser` 下沉到 `io::detail`
- 让 `LoadedMeshData` 升级成真正的内部构建边界
- 为 Phase D / E 的 API 清理和下游切换准备更干净的 I/O 入口

### 3.2 本阶段不做

- 不在 Phase C 内统一改 `io::save()` 的错误表达方式
- 不在 Phase C 内统一改 `TetMesh(path)` / `CubicMesh(path)` 公共签名
- 不在 Phase C 内做 `raw pointer -> std::span` 或 `std::optional` 化
- 不在 Phase C 内做最终的 public 目录重排

补充说明：

- 你允许破坏式切换，但 Phase C 的主收益来自 **I/O 结构拆分**
- 签名层面的横切 API 清理，放到后面单独一轮会更稳

## 4. Phase C 先固定的设计决策

### 4.1 `volumetricMeshIO.h` 只保留 façade

Phase C 结束后，[volumetricMeshIO.h](/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/volumetricMeshIO.h) 只应该暴露：

- `io::save`
- `io::save_to_ascii`
- `io::save_to_binary`
- `io::load_tet`
- `io::load_cubic`
- `io::detect_element_type`
- `io::detect_file_format`

不再继续暴露：

- `io::detail::LoadedMeshData`
- `io::detail::load_tet_data`
- `io::detail::load_cubic_data`
- `io::detail::detect_file_format_by_ext`

### 4.2 `LoadedMeshData` 升级为内部构建边界

Phase B 已经有：

- `internal::VolumetricMeshData`
- `internal::MaterialCatalog`

因此 Phase C 不应该继续让 `LoadedMeshData` 只是“一堆原始 vector”。建议改成内部 I/O 结果对象，例如：

```cpp
struct LoadedMeshData {
    VolumetricMesh::ElementType element_type;
    internal::VolumetricMeshData geometry;
    internal::MaterialCatalog material_catalog;
};
```

即使最终字段名略有不同，也要把方向固定住：

- reader 负责产出可直接喂给 `TetMesh` / `CubicMesh` 的内部对象
- `TetMesh::assignFromData()` / `CubicMesh::assignFromData()` 不再重做材料同步和几何装配

### 4.3 `VolumetricMeshParser` 只留在 `io::detail`

当前仓库里只有两处真正使用 parser：

- [volumetricMeshIO.cpp](/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/volumetricMeshIO.cpp)
- [tetMesh.cpp](/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/tetMesh.cpp) 的 `.ele/.node` 特殊构造路径

因此 Phase C 应该把它迁到：

```text
src/core/scene/volumetricMesh/io/detail/veg_parser.h/.cpp
```

模块外部不再 include 这个头。

### 4.4 特殊 `.ele/.node` 导入路径在 Phase C 一并收口

仓库内没有实际调用 [tetMesh.h](/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/tetMesh.h) 里的 `TetMesh(path, specialFileType, verbose)`。

因此 Phase C 可以直接做两种选择中的一个：

1. 删除这个构造函数，改成 `io::detail::load_tetgen_data(...)`
2. 保留 public 构造函数，但让它只做一层薄转发

建议选 **方案 1**，因为它更符合“协议感知路径进入 I/O 层”的目标。

## 5. 目标目录结构

建议在现有 `internal/` 之外，正式引入 `io/` 子目录：

```text
src/core/scene/volumetricMesh/
├── io/
│   ├── mesh_io_types.h
│   ├── mesh_format_detector.h/.cpp
│   ├── mesh_ascii_reader.h/.cpp
│   ├── mesh_binary_reader.h/.cpp
│   ├── mesh_ascii_writer.h/.cpp
│   ├── mesh_binary_writer.h/.cpp
│   ├── material_serde.h/.cpp
│   └── detail/
│       ├── veg_parser.h/.cpp
│       └── tetgen_reader.h/.cpp
├── internal/
│   ├── volumetric_mesh_data.h/.cpp
│   ├── material_catalog.h/.cpp
│   └── ...
├── volumetricMeshIO.h
├── volumetricMeshIO.cpp
├── tetMesh.cpp
└── cubicMesh.cpp
```

Phase C 结束后：

- `volumetricMeshIO.cpp` 保留，但变成薄 façade
- 绝大多数实际逻辑转移到 `io/`

## 6. 阶段划分

### 阶段 C1：先补 I/O 专项护栏

1. **补 I/O 结构化回归测试** `[M]` `[low-risk]`
   - 新增测试目录：
     - `tests/core/scene/volumetricMesh/io/mesh_format_detector_test.cpp`
     - `tests/core/scene/volumetricMesh/io/mesh_ascii_io_test.cpp`
     - `tests/core/scene/volumetricMesh/io/mesh_binary_io_test.cpp`
     - `tests/core/scene/volumetricMesh/io/material_serde_test.cpp`
   - 覆盖：
     - `detect_file_format` 的扩展名判定
     - `detect_element_type` 的 file / binary 路径
     - ENu / Orthotropic / MooneyRivlin 的 round-trip
     - 错误格式 / 非法 header 的失败路径
   - 依赖：无
   - 为什么：当前 Phase B 的护栏更偏行为，Phase C 需要更直接的 I/O 层保护

2. **给 `*INCLUDE` / parser 行为补一条最小回归** `[S]` `[low-risk]`
   - 如果保留 `VolumetricMeshParser` 的 include 语义，就必须为它留一条最小测试
   - 依赖：步骤 1
   - 为什么：parser 一旦下沉，最容易在“看似纯搬家”时丢行为

### 阶段 C2：抽出内部 I/O 类型边界

3. **把 `LoadedMeshData` 从 public header 收回到 `io/mesh_io_types.h`** `[M]` `[med-risk]`
   - 新增：
     - `src/core/scene/volumetricMesh/io/mesh_io_types.h`
   - 调整：
     - `TetMesh::assignFromData(...)`
     - `CubicMesh::assignFromData(...)`
     - `volumetricMeshIO.cpp`
   - 依赖：步骤 1
   - 为什么：只要 `LoadedMeshData` 还挂在 public header 上，I/O 内部就没有真正独立

4. **让 `LoadedMeshData` 直接承载 Phase B 的内部对象** `[M]` `[med-risk]`
   - 从“raw vectors + materials/sets/regions”切到“geometry + material_catalog”
   - 依赖：步骤 3
   - 为什么：否则 reader 仍然在用老数据模型，Phase B 的收益没有真正落到 I/O 上

### 阶段 C3：拆 detector 与 material serde

5. **抽出 `mesh_format_detector`** `[S]` `[low-risk]`
   - 新增：
     - `src/core/scene/volumetricMesh/io/mesh_format_detector.h`
     - `src/core/scene/volumetricMesh/io/mesh_format_detector.cpp`
   - 搬走：
     - `detect_file_format_by_ext`
     - `detect_file_format`
     - `detect_element_type(file)`
     - `detect_element_type(binary)`
     - header / magic 判定辅助逻辑
   - 依赖：步骤 3
   - 为什么：detector 是天然独立职责，不应该继续埋在 reader / writer 之间

6. **抽出 `material_serde`** `[M]` `[med-risk]`
   - 新增：
     - `src/core/scene/volumetricMesh/io/material_serde.h`
     - `src/core/scene/volumetricMesh/io/material_serde.cpp`
   - 搬走：
     - ENu / Orthotropic / MooneyRivlin 的 ASCII 解析
     - ENu / Orthotropic / MooneyRivlin 的 Binary 读写
   - 依赖：步骤 4
   - 为什么：reader / writer 不该自己知道每种材料的字段布局

### 阶段 C4：拆 ASCII reader / writer，并下沉 parser

7. **抽出 `mesh_ascii_reader`** `[L]` `[med-risk]`
   - 新增：
     - `src/core/scene/volumetricMesh/io/mesh_ascii_reader.h`
     - `src/core/scene/volumetricMesh/io/mesh_ascii_reader.cpp`
   - 搬走：
     - `.veg` 顶点 / 元素 / material / set / region 解析
     - `LoadedMeshData` 组装
   - 依赖：步骤 4、5、6
   - 为什么：ASCII reader 是当前 `volumetricMeshIO.cpp` 的最大噪声源之一

8. **把 `VolumetricMeshParser` 下沉为 `io::detail::veg_parser`** `[M]` `[low-risk]`
   - 新增：
     - `src/core/scene/volumetricMesh/io/detail/veg_parser.h`
     - `src/core/scene/volumetricMesh/io/detail/veg_parser.cpp`
   - 删除或弃用：
     - 模块根目录下的 `volumetricMeshParser.h/.cpp`
   - 依赖：步骤 7
   - 为什么：parser 是实现细节，不该继续作为模块主入口文件存在

9. **把 `.ele/.node` 导入路径搬到 `io::detail::tetgen_reader`** `[M]` `[med-risk]`
   - 新增：
     - `src/core/scene/volumetricMesh/io/detail/tetgen_reader.h`
     - `src/core/scene/volumetricMesh/io/detail/tetgen_reader.cpp`
   - 处理：
     - 删除 `TetMesh(path, specialFileType, verbose)` 或让它只做一层临时转发
   - 依赖：步骤 8
   - 为什么：这是协议感知路径，不该继续留在 `TetMesh` 构造函数里

10. **抽出 `mesh_ascii_writer`** `[M]` `[low-risk]`
    - 新增：
      - `src/core/scene/volumetricMesh/io/mesh_ascii_writer.h`
      - `src/core/scene/volumetricMesh/io/mesh_ascii_writer.cpp`
    - 搬走：
      - `.veg` 文本写出
      - set / region / material block 输出
    - 依赖：步骤 6
    - 为什么：writer 与 reader 的失败模式和维护成本完全不同，应分开

### 阶段 C5：拆 Binary reader / writer，并把 `volumetricMeshIO.cpp` 收成 façade

11. **抽出 `mesh_binary_reader`** `[M]` `[med-risk]`
    - 新增：
      - `src/core/scene/volumetricMesh/io/mesh_binary_reader.h`
      - `src/core/scene/volumetricMesh/io/mesh_binary_reader.cpp`
    - 搬走：
      - `read_exact`
      - binary header 解析
      - binary material 读取
      - file / span 两种 binary reader 路径
    - 依赖：步骤 4、5、6
    - 为什么：binary reader 与 ASCII reader 的关注点完全不同，混在一起只会继续拉高复杂度

12. **抽出 `mesh_binary_writer`** `[M]` `[low-risk]`
    - 新增：
      - `src/core/scene/volumetricMesh/io/mesh_binary_writer.h`
      - `src/core/scene/volumetricMesh/io/mesh_binary_writer.cpp`
    - 搬走：
      - `write_exact`
      - binary mesh / material / set / region 写出
    - 依赖：步骤 6
    - 为什么：让字节协议集中在一个地方，方便后续做错误表达和版本化

13. **把 `volumetricMeshIO.cpp` 收成薄 façade** `[M]` `[low-risk]`
    - 目标：
      - `save()` 只负责按扩展名分发到 writer
      - `load_tet()` / `load_cubic()` 只负责调 reader + 构造对象
      - `detect_*()` 只负责调 detector
    - 依赖：步骤 5、7、10、11、12
    - 为什么：到这一步，`volumetricMeshIO.cpp` 不该再包含 reader / writer / parser 细节

### 阶段 C6：收口 public 入口与下游依赖

14. **把 `volumetricMeshIO.h` 收成真正的 public I/O header** `[M]` `[med-risk]`
    - 删除 `detail::*` 声明
    - 让 `TetMesh.cpp` / `CubicMesh.cpp` 直接 include 新内部 I/O 头，而不是继续借 public header 走 detail
    - 依赖：步骤 3~13
    - 为什么：public header 里暴露 detail，是当前 I/O 模块边界最直接的泄漏

15. **同步清理仓库内 I/O 依赖点** `[M]` `[med-risk]`
    - 必查：
      - `/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/api/c/pgo_c.cpp`
      - `/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/tools/tetMesher/tetMesher.cpp`
      - `/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/tools/tetMesher/mshFileToVegFile.cpp`
      - `/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/tools/cubicMesher/cubicMesher.cpp`
      - `/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/tests/core/scene/volumetricMesh/tet_mesh_test.cpp`
      - `/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/tests/core/scene/volumetricMesh/cubic_mesh_test.cpp`
    - 依赖：步骤 14
    - 为什么：即使 public façade 名字不变，include 路径和错误处理也可能受影响

## 7. 测试布局建议

继续沿着 `src` 结构拆，不回到单一大文件：

```text
tests/core/scene/volumetricMesh/
├── tet_mesh_test.cpp
├── cubic_mesh_test.cpp
├── mesh_queries_test.cpp
├── io/
│   ├── mesh_format_detector_test.cpp
│   ├── mesh_ascii_io_test.cpp
│   ├── mesh_binary_io_test.cpp
│   ├── material_serde_test.cpp
│   └── tetgen_reader_test.cpp
└── internal/
    ├── material_catalog_test.cpp
    ├── mesh_subset_editor_test.cpp
    └── volumetric_mesh_data_test.cpp
```

建议保留两类测试分工：

- façade 行为测试：继续验证 round-trip 与构造路径
- I/O 单元测试：直接盯 `reader / writer / detector / serde`

## 8. 建议提交拆分

不要把 Phase C 混成一个提交。建议至少拆成下面 6 个提交：

1. `C1-io-tests`: 新增 I/O 专项测试
2. `C2-io-types`: 抽 `mesh_io_types`，收回 `LoadedMeshData`
3. `C3-detector-serde`: 抽 `mesh_format_detector` 与 `material_serde`
4. `C4-ascii-parser`: 抽 ASCII reader / writer，下沉 `veg_parser`
5. `C5-binary`: 抽 binary reader / writer
6. `C6-io-facade`: 把 `volumetricMeshIO.cpp/.h` 收成薄 façade，并清理 call sites

如果 `.ele/.node` 特殊构造函数也要删，建议单独再拆一个提交，不要夹在 detector 或 writer 提交里。

## 9. 构建与验证

Phase C 每个子阶段至少要跑：

```bash
cmake --build /Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/build/core-debug --target \
  core_scene_volumetric_mesh_test \
  core_energy_scene_to_simulation_mesh_test \
  core_energy_material_binding_test \
  api_pgo_c_test \
  cubicMesher_tool_test

ctest --test-dir /Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/build/core-debug \
  -R "core.scene.volumetric_mesh|core.energy.scene_to_simulation_mesh|core.energy.material_binding|api.pgo_c.basic|cubicMesher.tool.uniform" \
  --output-on-failure
```

如果新增了 `tests/core/scene/volumetricMesh/io/` 下的专门测试，还要把它们纳入同一个 `core_scene_volumetric_mesh_test` 或新增独立 target。

## 10. Phase C 退出条件

Phase C 完成时，至少要满足下面这些条件：

1. `volumetricMeshIO.cpp` 已经降为薄 façade，不再承载主要解析 / 写出逻辑
2. `volumetricMeshIO.h` 不再暴露 `detail::LoadedMeshData` 和 `load_*_data()`
3. `VolumetricMeshParser` 已下沉到 `io::detail`
4. ASCII / Binary / detector / material serde 已拆成独立翻译单元
5. `TetMesh.cpp` / `CubicMesh.cpp` 不再通过 public `volumetricMeshIO.h` 访问 detail 接口
6. Phase B 的最小验证集继续通过

建议再加一个结构性量化标准：

- `volumetricMeshIO.cpp` 目标缩到 `150` 行以内
- 任何单个新的 I/O `.cpp` 最好不要再次超过 `350` 行

如果拆完以后只是把 `volumetricMeshIO.cpp` 复制成五个 500 行文件，那说明只是“文件分裂”，不是“职责拆分”。

## 11. 下一阶段入口

Phase C 结束后，才进入 Phase D / E 这类真正会引起横切调用点切换的阶段：

- `raw pointer -> std::span`
- `-1 -> std::optional`
- `io::save` 错误表达升级
- 文件构造路径进一步收缩
- 下游 API / C API / tools 的统一切新接口

也就是说，Phase C 的价值是把 I/O 从“历史累积的协议大文件”整理成可持续演进的模块边界，让后面的破坏式 API 切换不是在一个 1000 行黑箱上动刀。
