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
from pypgo.solver.damping import (
    Damping,
    FixedDamping,
    NoDamping,
)
from pypgo.solver.line_search import (
    Backtrack,
    Brents,
    Golden,
    LineSearch,
    Simple,
)
from pypgo.solver.sparse_solver import (
    Auto,
    EigenLDLT,
    MKLPardiso,
    OrigPardiso,
    SparseSolver,
)
from pypgo.solver.termination import (
    AbsoluteTermination,
    HybridTermination,
    RelativeTermination,
    Termination,
)
from pypgo.solver.optimizer import NewtonOptimizer

__all__ = [
    "AbsoluteTermination",
    "Auto",
    "Backtrack",
    "Bounds",
    "Brents",
    "Damping",
    "HybridTermination",
    "EigenLDLT",
    "FixedDamping",
    "Golden",
    "LineSearch",
    "MKLPardiso",
    "NewtonOptimizer",
    "NoDamping",
    "OptimizationProblem",
    "Optimizer",
    "OrigPardiso",
    "RelativeTermination",
    "Simple",
    "SolveDiagnostics",
    "SolveStatus",
    "SolverResult",
    "SparseSolver",
    "Termination",
]
