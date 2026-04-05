# Folded Cubic Box Self IPC

This example is a self-contact `ipc-barrier` smoke/debug scene.

- The volumetric mesh is reused from [`../cubic-box/cubic-box.veg`](../cubic-box/cubic-box.veg).
- The surface mesh is a folded copy of `cubic-box.obj` that brings one layer close to another, so self near-contact pairs activate reliably.
- The goal is to exercise the self `ipc-barrier` runtime path and produce a nonzero `# self active pairs` log.

Current scope:

- This is not a polished physics showcase.
- The current self IPC path still does not include a self feasible alpha filter.
- Treat it as a regression/debug example for runtime integration.
