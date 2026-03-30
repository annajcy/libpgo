import argparse
import pypgo

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run simulation from a config JSON file.")
    parser.add_argument("config", help="Path to simulation config JSON")
    parser.add_argument(
        "--deterministic",
        action="store_true",
        help="Force deterministic mode (single-threaded) regardless of config file setting.",
    )

    args = parser.parse_args()

    deterministic = True if args.deterministic else None
    print(pypgo.run_sim_from_config(args.config, deterministic=deterministic))
