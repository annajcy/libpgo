# `pypgo/fem/mesh.py` — 仿真网格与壳配置 I/O

> 源文件：`pypgo/fem/mesh.py`（87 行）。模块架构见 [overview.md](overview.md)。
>
> `SimulationMesh` 是几何世界（[../mesh/overview.md](../mesh/overview.md)）到求解世界的**唯一入口**：所有 FEM 装配（[energy.md](energy.md)）、质量算子（[formulations.md](formulations.md)）都从它出发。

## class `SimulationMesh`

C++ `PySimulationMesh` 的只读句柄。不直接构造——用两个显式工厂方法，分别对应两类离散域：

### classmethod `create_volumetric(volume_mesh)`

```python
SimulationMesh.create_volumetric(volume_mesh: VolumeMesh) -> SimulationMesh
```

从 [`VolumeMesh`](../mesh/volume/core.md)（tet 或 cubic，**含 VEG 材料区域**）创建体仿真网格。材料信息随网格进入 C++（`_core.create_simulation_mesh_from_volume`）——之后本构工厂从中烘焙每元素的 $(\mu,\lambda)$（见 [elastic.md](elastic.md) 的参数来源说明）。

### classmethod `create_shell(surface, material)`

```python
SimulationMesh.create_shell(surface: TriMeshData,
                            material: KoiterStVKShellMaterial) -> SimulationMesh
```

从三角面网格 + 壳材料创建壳仿真网格（`_core.create_simulation_mesh_from_shell`，传入 `thickness`、`E_membrane`、`nu_membrane` 三个标量）。这三个值是 Koiter 壳 5 通道参数场的**播种值**（[fields.md](fields.md) 的 `values=None` 路径）。

### 属性 `mesh_type`

字符串：网格类别（tet / cubic / shell），决定哪些 formulation 合法。

### 属性 `num_vertices` / `num_elements` / `num_element_vertices`

顶点数、单元数、每单元顶点数（tet=4、cubic=8、shell=3）。注意 `num_vertices` 是**几何顶点数**；总 DOF 数取决于 formulation（线性 formulation 为 $3n_v$，tricubic Hermite 为 $24n_v$）——以 [`DeformationEnergy.num_dofs`](energy.md) 为准。

---

## func `write_shell_config(path, surface, material)`

把壳场景（面网格 + 材料）持久化为一对文件：

- `<path去后缀>.obj` —— 面网格（经 [`write_obj`](../mesh/overview.md)）
- `<path>` —— JSON：`{"mesh_obj": "<obj文件名>", "material": {"kind": "KoiterStVKShellMaterial", ...字段}}`

JSON 中 `mesh_obj` 存**文件名**（相对所在目录），所以这对文件可整体搬移。

## func `read_shell_config(path)`

```python
read_shell_config(path) -> (TriMeshData, KoiterStVKShellMaterial)
```

逆操作：读 JSON、按 `mesh_obj` 在同目录找 OBJ、校验 `kind == "KoiterStVKShellMaterial"`（其他类型抛 `ValueError`），返回可直接喂给 `create_shell` 的二元组。

---

## 用法示例

```python
import pypgo

# 体
veg = pypgo.mesh.read_veg("box.veg")
volume = pypgo.mesh.VolumeMesh.from_veg_file(veg)
sm = pypgo.fem.SimulationMesh.create_volumetric(volume)
sm.mesh_type, sm.num_vertices, sm.num_element_vertices

# 壳（含持久化往返）
mat = pypgo.fem.KoiterStVKShellMaterial(thickness=1e-3, E_membrane=1e6, nu_membrane=0.4)
pypgo.fem.write_shell_config("scene.shell.json", surface, mat)
surface2, mat2 = pypgo.fem.read_shell_config("scene.shell.json")
shell_sm = pypgo.fem.SimulationMesh.create_shell(surface2, mat2)
```

## 交叉链接

- 上游几何：[../mesh/volume/core.md](../mesh/volume/core.md)（`VolumeMesh`）、[../mesh/data.md](../mesh/data.md)（`TriMeshData`）
- 下游消费：[energy.md](energy.md)、[formulations.md](formulations.md)、[../energy/attachment.md](../energy/attachment.md)（`VertexAttachment(sim_mesh=...)`）
- 壳材料字段：[elastic.md](elastic.md)（`KoiterStVKShellMaterial`）
