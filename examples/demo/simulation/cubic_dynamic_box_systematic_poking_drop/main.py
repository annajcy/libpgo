"""Drop a cubic box onto a penalty floor using Systematic Poking material."""

import argparse
from importlib.util import find_spec
from pathlib import Path

import numpy as np
import pypgo as pgo
import pypgo.animation as pa
import pypgo.contact as pc
import pypgo.energy as pe
import pypgo.fem as pf
import pypgo.sim as pgs
import pypgo.solver as ps


CASE_DIR = Path(__file__).resolve().parent
EXAMPLES_DIR = CASE_DIR.parents[2]
ASSET_DIR = EXAMPLES_DIR / "assets"
OUTPUT_DIR = CASE_DIR / "output"
TIMESTEP = 2.0e-3
NUM_STEPS = 200
DUMP_INTERVAL = 20


def _material(mesh, *, youngs_modulus=2.0e5, poisson_ratio=0.35):
    # Log-uniform paper-style knots. Both arrays contain the rest knot 1.
    stretch_knots = np.exp(np.linspace(np.log(0.5), np.log(2.0), 17))
    volume_knots = np.exp(np.linspace(-1.0, 1.0, 17))
    stretch_rest_knot_index = len(stretch_knots) // 2
    volume_rest_knot_index = len(volume_knots) // 2

    elastic = pf.SystematicPokingDefinition(
        stretch_knots,
        stretch_rest_knot_index,
        volume_knots,
        volume_rest_knot_index,
    )
    plastic = pf.VolumetricPlasticityDefinition(dofs=0)
    mu = youngs_modulus / (2.0 * (1.0 + poisson_ratio))
    lame_lambda = (
        youngs_modulus
        * poisson_ratio
        / ((1.0 + poisson_ratio) * (1.0 - 2.0 * poisson_ratio))
    )
    # Curvature samples of
    # f(s) = mu/2 * (s^2 - 1) - mu * log(s), whose f(1)=f'(1)=0.
    stretch_curvatures = mu * (1.0 + 1.0 / stretch_knots**2)
    elastic_parameters = np.concatenate(
        [stretch_curvatures, [lame_lambda]])
    elementwise_parameters = np.broadcast_to(
        elastic_parameters,
        (mesh.num_elements, elastic.num_optimizable_channels),
    ).copy()

    binding = pf.MaterialBinding(
        pf.ElasticMaterialBinding(
            elastic, mesh.num_elements,
            np.empty((mesh.num_elements, 0), dtype=np.float64)),
        pf.PlasticMaterialBinding(
            plastic, mesh.num_elements,
            np.empty((mesh.num_elements, 0), dtype=np.float64)),
    )
    return binding, pf.MaterialState(elementwise_parameters, np.empty(0))


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--num-steps", type=int, default=NUM_STEPS)
    parser.add_argument("--dump-interval", type=int, default=DUMP_INTERVAL)
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR)
    parser.add_argument("--no-abc", action="store_true")
    parser.add_argument("--no-visualize", action="store_true")
    args = parser.parse_args(argv)
    if args.num_steps < 1:
        parser.error("--num-steps must be positive")
    if args.dump_interval < 1:
        parser.error("--dump-interval must be positive")

    volume = pgo.mesh.volume.VolumeMesh(
        pgo.mesh.volume.read_veg(
            str(ASSET_DIR / "veg" / "cubic" / "box.veg"))
    )
    surface = volume.extract_surface_mesh()
    mesh = pf.SimulationMesh(volume)
    formulation = pf.CubicLinear()

    material_binding, material_state = _material(mesh)
    deformation_operator = pf.DeformationEnergyOperator(
        mesh, material_binding,
        formulation=formulation,
    )
    deformation = pf.DeformationPotentialEnergy(
        deformation_operator, material_state)

    density = 1000.0
    mass = formulation.mass_matrix(mesh, density)
    gravity_force = formulation.body_force(
        mesh,
        np.array([0.0, -9.81, 0.0]),
        density,
    )

    surface_map = formulation.surface_embedding_matrix(
        volume, surface.vertices)
    contact_surface = pc.ContactSurface.embedded(
        surface.vertices, surface_map)
    floor = pc.FloorEnergy(
        contact_surface,
        axis="y",
        side="keep_above",
        height=0.0,
        stiffness=2.0e5,
    )

    state = pgs.DynamicState(
        displacement=np.zeros(deformation.num_dofs),
        velocity=np.zeros(deformation.num_dofs),
        acceleration=np.zeros(deformation.num_dofs),
    )
    simulation = pgs.DynamicSimulation(
        mass=mass,
        energy=pe.EnergySet([
            (deformation, 1.0),
            (floor, 1.0),
        ]),
        state=state,
        timestep=TIMESTEP,
        damping=(1.0e-2, 1.0e-4),
        integrator=pgs.BackwardEulerDynamicStepper(),
    )
    optimizer = ps.NewtonOptimizer(
        max_iterations=100,
        gradient_tolerance=1.0e-5,
        line_search=ps.Backtrack(),
    )

    args.output_dir.mkdir(parents=True, exist_ok=True)
    final_surface = surface
    pgo.mesh.write_obj(
        str(args.output_dir / "surface0000.obj"), surface)
    write_abc = not args.no_abc and pa.has_animation_io()
    if not args.no_abc and not write_abc:
        print("Alembic I/O is not available; skipping animation.abc.")
    abc_displacements = (
        [np.zeros(surface.vertices.size, dtype=np.float64)]
        if write_abc
        else []
    )

    for step in range(1, args.num_steps + 1):
        frame = simulation.step(
            external_force=gravity_force,
            optimizer=optimizer,
        )
        if not frame.accepted:
            raise RuntimeError(
                f"frame {frame.frame_index} rejected: "
                f"{frame.solver_result.status.name}"
            )
        surface_displacement = np.asarray(
            surface_map @ frame.displacement,
            dtype=np.float64,
        ).reshape((-1, 3))
        if write_abc:
            abc_displacements.append(
                np.ascontiguousarray(surface_displacement).ravel())
        if (
            step % args.dump_interval == 0
            or step == args.num_steps
        ):
            vertices = surface.vertices + surface_displacement
            final_surface = pgo.mesh.TriMeshData(
                vertices, surface.elements)
            pgo.mesh.write_obj(
                str(args.output_dir / f"surface{step:04d}.obj"),
                final_surface,
            )
            print(
                f"step {step:4d}  "
                f"iterations={frame.solver_result.iterations:2d}  "
                f"min_y={vertices[:, 1].min():.6f}"
            )

    if write_abc:
        abc_path = args.output_dir / "animation.abc"
        pa.AbcWriter.dump(
            abc_path,
            "systematic_poking",
            rest_positions=surface.vertices.ravel(),
            triangles=surface.elements,
            displacements=abc_displacements,
            fps=1.0 / TIMESTEP,
        )
        print("saved Alembic cache ->", abc_path)

    if args.no_visualize:
        pass
    elif find_spec("pyvista") is None:
        print("PyVista is not installed; skipping visualization.")
    else:
        floor_visual = pgo.mesh.TriMeshData(
            np.array([
                [-0.6, 0.0, -0.6],
                [0.6, 0.0, -0.6],
                [0.6, 0.0, 0.6],
                [-0.6, 0.0, 0.6],
            ]),
            np.array([[0, 1, 2], [0, 2, 3]]),
        )
        pgo.mesh.plot_surface(
            [surface, final_surface, floor_visual],
            titles=["rest", "final", "floor"],
            colors=["lightgray", "salmon", "steelblue"],
            show_edges=False,
            window_size=(1200, 420),
        )

    print("saved frames ->", args.output_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
