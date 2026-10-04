"""`mqt-scpd` on the overlay binding under artifacts/dev/py rather than the installed one.

The venv's editable install puts a scikit-build finder first on `sys.meta_path`, which maps
`mqt.scpd.pyscpd` to the binding in site-packages whatever PYTHONPATH says; so that finder is
taken out here and the overlay put first on the path. Used by dev-run.sh only.
"""

import os
import sys

sys.meta_path[:] = [finder for finder in sys.meta_path if "ScikitBuild" not in type(finder).__name__]
sys.path.insert(0, os.environ.get("SCPD_DEV_OVERLAY", "artifacts/dev/py"))

from mqt.scpd.cli import main  # noqa: E402

sys.exit(main())
