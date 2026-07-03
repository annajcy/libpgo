"""pypgo-sim-shell-static — static shell FEM solve from JSON/CLI scene config."""

from pypgo.tools.sim._cli import run_cli


def main(argv=None) -> int:
    return run_cli(mesh_type="shell", mode="static",
                   prog="pypgo-sim-shell-static", argv=argv)


if __name__ == "__main__":
    raise SystemExit(main())
