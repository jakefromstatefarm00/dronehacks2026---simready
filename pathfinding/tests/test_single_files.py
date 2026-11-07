"""The generated FlyX single files must match their sources (rebuild: python tools/build_single_file.py)."""
import importlib.util
from pathlib import Path
import re

import pytest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("build_single_file", ROOT / "tools/build_single_file.py")
build = importlib.util.module_from_spec(spec)
spec.loader.exec_module(build)


def _strip_commit(text):
    return re.sub(r"commit \w+\)", "commit)", text)


@pytest.fixture(scope="module")
def course():
    return build.build_map(.9)


@pytest.mark.parametrize("name, fn", [("upload_to_flyx.py", "build_native"), ("pathplan_mission.py", "build_skybeta")])
def test_generated_file_is_up_to_date(course, name, fn):
    fresh = getattr(build, fn)(.9, *course, "x")
    on_disk = (ROOT / "flyx" / name).read_text(encoding="utf-8")
    assert _strip_commit(on_disk) == _strip_commit(fresh), f"flyx/{name} is stale: run python tools/build_single_file.py"
