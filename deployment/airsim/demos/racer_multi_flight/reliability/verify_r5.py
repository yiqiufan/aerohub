"""Check the preserved R5 recovery evidence after the simulator has stopped."""
import argparse
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

from r5_stop import matches


def read(path):
    return json.loads(path.read_text(encoding="utf-8"))


def verify(root):
    index = read(Path(__file__).resolve().parents[1] / "pc-dual-validation.json")
    expected = index["cases"]["R5"]["artifacts_sha256"]
    errors = []
    for relative, digest in expected.items():
        path = root / relative
        if not path.is_file():
            errors.append("missing: " + relative)
            continue
        with path.open("rb") as stream:
            if hashlib.file_digest(stream, "sha256").hexdigest() != digest:
                errors.append("changed: " + relative)
    if errors:
        return errors
    local = read(root / "local.json")
    owner = read(root / "injection-owner.json")
    name = local["project"] + "-native"
    states = read(root / "cleanup-result.json")
    state = next(row["State"] for row in states if row["name"] == name)
    # Reconstruct the stopped container's identity from this archived run.
    # Live Docker ownership and landed state were checked by the original runner.
    observed = dict(Id=owner["container"], Name="/" + name, State=state,
                    Config={"Labels": {"aerohub.racer.owner": local["project"]}})
    deployment = SimpleNamespace(root=root, prefix=local["project"], local=local,
                                 fleet=read(root / "configuration/fleet.json"))
    if not matches(deployment, observed):
        errors.append("recorded exits or recovery differ from the R5 contract")
    return errors


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-root", type=Path, required=True)
    issues = verify(parser.parse_args().run_root)
    print(json.dumps({"evidence_matches": not issues, "errors": issues}, indent=2))
    raise SystemExit(bool(issues))
