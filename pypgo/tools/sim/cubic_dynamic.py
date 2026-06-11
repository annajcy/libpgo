"""pypgo-sim-cubic-dynamic — dynamic cubic FEM solve from JSON/CLI scene config."""

from pypgo.tools.sim._cli import run_cli


def main(argv=None) -> int:
    return run_cli(mesh_type="cubic", mode="dynamic",
                   prog="pypgo-sim-cubic-dynamic", argv=argv)


if __name__ == "__main__":
    raise SystemExit(main())
