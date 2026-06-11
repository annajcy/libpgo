"""pypgo-sim-tet-static — static tet FEM solve from JSON/CLI scene config."""

from pypgo.tools.sim._cli import run_cli


def main(argv=None) -> int:
    return run_cli(mesh_type="tet", mode="static",
                   prog="pypgo-sim-tet-static", argv=argv)


if __name__ == "__main__":
    raise SystemExit(main())
