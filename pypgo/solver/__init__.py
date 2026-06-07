"""pypgo.solver - optimization problem and optimizer entry points.

The result and problem types are the stable data layer; concrete optimizers
live in ``optimizer.py`` and are the side expected to grow over time.  This
package re-exports a flat public surface, so ``pypgo.solver.NewtonOptimizer``
and friends keep working unchanged.
"""

from pypgo.solver.result import (
    SolveDiagnostics,
    SolveStatus,
    SolverResult,
)
from pypgo.solver.problem import (
    Bounds,
    OptimizationProblem,
)
from pypgo.solver.base import Optimizer
from pypgo.solver.optimizer import NewtonOptimizer

__all__ = [
    "Bounds",
    "NewtonOptimizer",
    "OptimizationProblem",
    "Optimizer",
    "SolveDiagnostics",
    "SolveStatus",
    "SolverResult",
]
