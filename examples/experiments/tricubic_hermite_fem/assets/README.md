# Experiment Assets

The static and dynamic experiments share this asset tree:

```text
assets/
├── fixed/       # static attachment patches
├── obj/         # source surfaces, ground obstacle, generated boundaries
└── veg/
    ├── cubic/   # base, subdiv2, and subdiv3 cubic meshes
    └── tet/     # tuned same-domain tet references
```

Tracked source assets:

- `obj/bunny.obj`
- `obj/dragon.obj`
- `obj/bottom.1.obj`
- `fixed/bunny-surface-fixed-ear-tip.txt`
- `fixed/dragon-surface-fixed.txt`

Generated assets are ignored by Git:

- `obj/*-conservative-r15-surface.obj`
- `veg/cubic/*.veg`
- `veg/tet/*.veg`

Generate all required assets with `../run_experiments.sh`, or use the utilities
under `../mesh/` individually.
