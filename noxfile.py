#!/usr/bin/env -S uv run --script --quiet
# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

# /// script
# dependencies = ["nox"]
# ///

"""Nox sessions."""

from __future__ import annotations

import argparse
import contextlib
import os
import shutil
import tempfile
from pathlib import Path
from typing import TYPE_CHECKING

import nox

if TYPE_CHECKING:
    from collections.abc import Generator, Sequence

nox.needs_version = ">=2025.10.16"
nox.options.default_venv_backend = "uv"


PYTHON_ALL_VERSIONS = ["3.11", "3.12", "3.13", "3.14"]

SCHEMA_DIR = Path("schemas")
GENERATED_CPP_DIR = Path("include") / "mqt-scpd" / "generated"
GENERATED_PYTHON_DIR = Path("python") / "mqt" / "scpd" / "generated"
# Python reads and writes the chip, the configuration and the stage artifacts itself. The DRC report
# is written as JSON, which Python reads without generated code.
PYTHON_ONLY_CPP_SCHEMAS = {"drc.fbs"}

if os.environ.get("CI", None):
    nox.options.error_on_missing_interpreters = True


@contextlib.contextmanager
def preserve_lockfile() -> Generator[None]:
    """Preserve the lockfile by moving it to a temporary directory."""
    with tempfile.TemporaryDirectory() as temp_dir_name:
        shutil.move("uv.lock", f"{temp_dir_name}/uv.lock")
        try:
            yield
        finally:
            shutil.move(f"{temp_dir_name}/uv.lock", "uv.lock")


@nox.session(reuse_venv=True, default=True)
def lint(session: nox.Session) -> None:
    """Run the linter."""
    if shutil.which("prek") is None:
        session.install("prek")

    session.run("prek", "run", "--all-files", *session.posargs, external=True)


def _run_tests(
    session: nox.Session,
    *,
    install_args: Sequence[str] = (),
    extra_command: Sequence[str] = (),
    pytest_run_args: Sequence[str] = (),
) -> None:
    env = {"UV_PROJECT_ENVIRONMENT": session.virtualenv.location}
    if shutil.which("cmake") is None and shutil.which("cmake3") is None:
        session.install("cmake")
    if shutil.which("ninja") is None:
        session.install("ninja")

    # install build and test dependencies on top of the existing environment
    session.run(
        "uv",
        "sync",
        "--inexact",
        "--only-group",
        "build",
        "--only-group",
        "test",
        *install_args,
        env=env,
    )
    session.run(
        "uv",
        "sync",
        "--inexact",
        "--no-dev",  # do not auto-install dev dependencies
        "--no-build-isolation-package",
        "mqt-scpd",  # build the project without isolation
        *install_args,
        env=env,
    )
    if extra_command:
        session.run(*extra_command, env=env)
    session.run(
        "uv",
        "run",
        "--no-sync",  # do not sync as everything is already installed
        *install_args,
        "pytest",
        *pytest_run_args,
        *session.posargs,
        "--cov-config=pyproject.toml",
        env=env,
    )


@nox.session(python=PYTHON_ALL_VERSIONS, reuse_venv=True, default=True)
def tests(session: nox.Session) -> None:
    """Run the test suite."""
    _run_tests(session)


@nox.session(python=PYTHON_ALL_VERSIONS, reuse_venv=True, venv_backend="uv", default=True)
def minimums(session: nox.Session) -> None:
    """Test the minimum versions of dependencies."""
    with preserve_lockfile():
        _run_tests(
            session,
            install_args=["--resolution=lowest-direct"],
            pytest_run_args=["-Wdefault"],
        )
        env = {"UV_PROJECT_ENVIRONMENT": session.virtualenv.location}
        session.run("uv", "tree", "--frozen", env=env)


@nox.session(reuse_venv=True)
def docs(session: nox.Session) -> None:
    """Build the docs. Use "--non-interactive" to avoid serving. Pass "-b linkcheck" to check links."""
    parser = argparse.ArgumentParser()
    parser.add_argument("-b", dest="builder", default="html", help="Build target (default: html)")
    args, posargs = parser.parse_known_args(session.posargs)

    serve = args.builder == "html" and session.interactive
    if serve:
        session.install("sphinx-autobuild")

    env = {"UV_PROJECT_ENVIRONMENT": session.virtualenv.location}
    # install build and docs dependencies on top of the existing environment
    session.run(
        "uv",
        "sync",
        "--inexact",
        "--only-group",
        "build",
        "--only-group",
        "docs",
        env=env,
    )

    # build the C++ API docs using doxygen
    with session.chdir("docs"):
        if shutil.which("doxygen") is None:
            session.error("doxygen is required to build the C++ API docs")

        Path("_build/doxygen").mkdir(parents=True, exist_ok=True)
        session.run("doxygen", "Doxyfile", external=True)
        Path("api/cpp").mkdir(parents=True, exist_ok=True)
        session.run(
            "breathe-apidoc",
            "-o",
            "api/cpp",
            "-m",
            "-f",
            "-g",
            "namespace",
            "_build/doxygen/xml/",
            external=True,
        )

    shared_args = [
        "-n",  # nitpicky mode
        "-T",  # full tracebacks
        f"-b={args.builder}",
        "docs",
        f"docs/_build/{args.builder}",
        *posargs,
    ]

    session.run(
        "uv",
        "run",
        "--no-dev",  # do not auto-install dev dependencies
        "--no-build-isolation-package",
        "mqt-scpd",  # build the project without isolation
        "sphinx-autobuild" if serve else "sphinx-build",
        *shared_args,
        env=env,
    )


def _find_flatc(build_dir: Path) -> Path:
    """Locate the flatc executable that the C++ build produced.

    Returns:
        The path to the executable.

    Raises:
        FileNotFoundError: If the build directory holds no flatc executable.
    """
    candidates = [
        path
        for path in (build_dir / "_deps" / "flatbuffers-build").glob("flatc*")
        if path.is_file() and path.suffix in {"", ".exe"}
    ]
    if not candidates:
        msg = f"flatc was not built under {build_dir}"
        raise FileNotFoundError(msg)
    return candidates[0].resolve()


@nox.session(reuse_venv=True, venv_backend="uv")
def schemas(session: nox.Session) -> None:
    """Regenerate the C++ and Python code from the FlatBuffers schemas.

    The session builds ``flatc`` from the FlatBuffers source that the C++ build fetches, so the
    generator always matches the runtime. Pass ``--check`` to fail when the committed code is stale.
    """
    check = "--check" in session.posargs
    if shutil.which("cmake") is None and shutil.which("cmake3") is None:
        session.install("cmake")
    if shutil.which("ninja") is None:
        session.install("ninja")

    build_dir = Path("build") / "schemas"
    session.run(
        "cmake",
        "-S",
        ".",
        "-B",
        str(build_dir),
        "-G",
        "Ninja",
        "-DCMAKE_BUILD_TYPE=Release",
        "-DMQT_SCPD_BUILD_FLATC=ON",
        "-DBUILD_MQT_SCPD_TESTS=OFF",
        external=True,
    )
    session.run("cmake", "--build", str(build_dir), "--target", "flatc", external=True)
    flatc = _find_flatc(build_dir)

    # Remove the output of schemas and types that no longer exist before regenerating.
    for stale in [
        *GENERATED_CPP_DIR.glob("*_generated.hpp"),
        *(path for path in GENERATED_PYTHON_DIR.glob("*.py*") if path.name != "__init__.py"),
    ]:
        stale.unlink()

    schema_names = sorted(path.name for path in SCHEMA_DIR.glob("*.fbs"))
    with session.chdir(SCHEMA_DIR):
        session.run(
            str(flatc),
            "--cpp",
            "--gen-object-api",
            "--gen-compare",
            "--scoped-enums",
            "--cpp-std",
            "c++17",
            "--filename-suffix",
            "_generated",
            "--filename-ext",
            "hpp",
            "--include-prefix",
            "mqt-scpd/generated",
            "--warnings-as-errors",
            "-o",
            str(Path("..") / GENERATED_CPP_DIR),
            *schema_names,
            external=True,
        )

    # flatc writes one Python module per type into the directories of the schema namespace, together
    # with empty package initializers up to the top-level namespace and a stub per module. Only the
    # modules are wanted; they keep their inline annotations.
    python_schema_names = [name for name in schema_names if name not in PYTHON_ONLY_CPP_SCHEMAS]
    with tempfile.TemporaryDirectory() as temp_dir_name, session.chdir(SCHEMA_DIR):
        session.run(
            str(flatc),
            "--python",
            "--gen-object-api",
            "--python-typing",
            "--python-decode-obj-api-strings",
            "--warnings-as-errors",
            "-o",
            temp_dir_name,
            *python_schema_names,
            external=True,
        )
        generated = Path(temp_dir_name) / "mqt" / "scpd" / "generated"
        for module in sorted(generated.glob("*.py")):
            if module.name != "__init__.py":
                shutil.copyfile(module, Path("..") / GENERATED_PYTHON_DIR / module.name)

    if check:
        status = session.run(
            "git",
            "status",
            "--porcelain",
            "--",
            str(GENERATED_CPP_DIR),
            str(GENERATED_PYTHON_DIR),
            external=True,
            silent=True,
        )
        if status and status.strip():
            session.log(status)
            session.error("The committed schema-generated code is stale. Run `uvx nox -s schemas` and commit it.")


@nox.session(reuse_venv=True, venv_backend="uv")
def stubs(session: nox.Session) -> None:
    """Generate type stubs for Python bindings using nanobind."""
    env = {"UV_PROJECT_ENVIRONMENT": session.virtualenv.location}
    session.run(
        "uv",
        "sync",
        "--no-dev",
        "--group",
        "build",
        env=env,
    )

    package_root = Path(__file__).parent / "python" / "mqt" / "scpd"

    session.run(
        "python",
        "-m",
        "nanobind.stubgen",
        "--recursive",
        "--include-private",
        "--output-dir",
        str(package_root),
        "--module",
        "mqt.scpd.pyscpd",
    )

    # The schema-generated stubs are owned by the schemas session.
    pyi_files = [path for path in package_root.glob("**/*.pyi") if path.parent != package_root / "generated"]

    if not pyi_files:
        session.warn("No .pyi files found")
        return

    if shutil.which("prek") is None:
        session.install("prek")

    # Allow both 0 (no issues) and 1 as success codes for fixing up stubs
    success_codes = [0, 1]
    session.run("prek", "run", "license-tools", "--files", *pyi_files, external=True, success_codes=success_codes)
    session.run("prek", "run", "ruff-check", "--files", *pyi_files, external=True, success_codes=success_codes)
    session.run("prek", "run", "ruff-format", "--files", *pyi_files, external=True, success_codes=success_codes)

    # Run ruff-check again to ensure everything is clean
    session.run("prek", "run", "ruff-check", "--files", *pyi_files, external=True)


if __name__ == "__main__":
    nox.main()
