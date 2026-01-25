import importlib.util
import sys
from pathlib import Path

from subsystems.intake.io import IntakeIO, IntakeIOTalonFX, IntakeIOSim

# Import IntakeSubsystem from hyphenated filename
from pathlib import Path
import importlib.util
import sys

_intake_subsystem_path = Path(__file__).parent / "intake_subsystem.py"

spec = importlib.util.spec_from_file_location(
    "intake_subsystem",
    _intake_subsystem_path
)

if spec is None or spec.loader is None:
    raise ImportError(f"Could not load intake subsystem from {_intake_subsystem_path}")

_intake_subsysten_module = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = _intake_subsystem_module
spec.loader.exec_module(_intake_subsystem_module)

IntakeSubsystem = _intake_subsystem_module.IntakeSubsystem

__all__ = ["IntakeSubsystem"]
