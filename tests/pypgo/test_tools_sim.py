from pathlib import Path

import numpy as np

import pypgo as pgo
import pypgo.sim as tsim


FIXTURE_ROOT = Path(__file__).resolve().parents[1] / "fixtures" / "legacy" / "cubic" / "box"


def _cubic_box_volume_and_surface():
    volume = pgo.mesh.volume.VolumeMesh.from_veg_file(
        pgo.mesh.volume.read_veg(str(FIXTURE_ROOT / "box.veg"))
    )
    surface = pgo.mesh.read_obj(str(FIXTURE_ROOT / "box.obj"))
    return volume, surface


def test_core_sim_builder_api_is_public():
    assert tsim.RuntimeConfig is not None
    assert tsim.VolumeIPCSimulationSpec is not None


def test_build_volume_ipc_simulation_exposes_dynamic_runner():
    volume, surface = _cubic_box_volume_and_surface()
    spec = tsim.VolumeIPCSimulationSpec(
        volume=volume,
        surface=surface,
        runtime=tsim.RuntimeConfig(
            timestep=0.001,
            num_steps=2,
            gravity=(0.0, -9.81, 0.0),
            initial_velocity=(0.0, 0.0, 0.0),
            solver_max_iterations=20,
            solver_gradient_tolerance=1e-5,
        ),
        contact=tsim.IPCContactSpec(
            parameters=pgo.contact.IPCParameters(
                dhat=0.005,
                dhat_external=0.005,
                kappa=100.0,
            ),
        ),
        floors=[
            tsim.FloorSpec(
                axis="y",
                side="keep_above",
                height=float(surface.vertices[:, 1].min() - 0.1),
                stiffness=1000.0,
            )
        ],
    )

    runner = tsim.build_volume_ipc_simulation(spec)

    assert runner.num_dofs == volume.num_vertices * 3
    assert runner.surface.num_vertices == surface.num_vertices
    assert runner.surface_displacement().shape == surface.vertices.shape
    assert runner.deformed_surface().num_vertices == surface.num_vertices
    assert runner.energy.num_terms == 3


def test_volume_ipc_simulation_accepts_explicit_formulation():
    volume, surface = _cubic_box_volume_and_surface()
    spec = tsim.VolumeIPCSimulationSpec(
        volume=volume,
        surface=surface,
        formulation=pgo.fem.TricubicHermite(),
        runtime=tsim.RuntimeConfig(
            timestep=0.001,
            gravity=(0.0, -9.81, 0.0),
            initial_velocity=(0.0, -1.0, 0.0),
            solver_max_iterations=20,
            solver_gradient_tolerance=1e-5,
        ),
        contact=tsim.IPCContactSpec(
            parameters=pgo.contact.IPCParameters(
                dhat=0.005,
                dhat_external=0.005,
                kappa=100.0,
            ),
        ),
    )

    runner = tsim.build_volume_ipc_simulation(spec)

    assert runner.num_dofs == volume.num_vertices * 24
    assert runner.build.mass.shape == (runner.num_dofs, runner.num_dofs)
    assert runner.build.external_force.shape == (runner.num_dofs,)
    assert runner.build.contact_surface.num_simulation_dofs == runner.num_dofs
    assert runner.state.velocity.reshape((-1, 24))[:, :3].shape == (volume.num_vertices, 3)


def test_volume_ipc_runner_steps_and_maps_surface_displacement():
    volume, surface = _cubic_box_volume_and_surface()
    spec = tsim.VolumeIPCSimulationSpec(
        volume=volume,
        surface=surface,
        runtime=tsim.RuntimeConfig(
            timestep=0.001,
            num_steps=1,
            gravity=(0.0, -9.81, 0.0),
            solver_max_iterations=40,
            solver_gradient_tolerance=1e-5,
        ),
        contact=tsim.IPCContactSpec(
            parameters=pgo.contact.IPCParameters(
                dhat=0.005,
                dhat_external=0.005,
                kappa=100.0,
            ),
        ),
    )
    runner = tsim.build_volume_ipc_simulation(spec)

    frame = runner.step()

    assert frame.accepted
    assert runner.state.timestep_id == 1
    surface_displacement = runner.surface_displacement(frame.displacement)
    assert surface_displacement.shape == surface.vertices.shape
    assert np.linalg.norm(surface_displacement) > 0.0
