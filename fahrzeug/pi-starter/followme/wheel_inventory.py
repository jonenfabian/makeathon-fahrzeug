"""Offline-Wheel-/Markerprüfung für einen expliziten Zielkandidaten, nie installieren.

Nur gehashte Archive lesen, keine nativen Module importieren, kein Netzwerk/USB.
packaging (Apache-2.0 OR BSD-2-Clause) gehört zum optionalen Audit-Werkzeug.
"""

from __future__ import annotations

import argparse
import base64
import csv
import hashlib
import io
import json
import re
import zipfile
from email.parser import BytesParser
from pathlib import Path, PurePosixPath

from packaging.requirements import Requirement
from packaging.specifiers import SpecifierSet
from packaging.utils import canonicalize_name, parse_wheel_filename

from .elf_inventory import dynamic_inventory, native_candidates
from .license_bundle import digest, is_notice

ENVIRONMENT_KEYS = {"implementation_name", "implementation_version", "os_name", "platform_machine", "platform_release",
                    "platform_system", "platform_version", "python_full_version", "platform_python_implementation",
                    "python_version", "sys_platform"}


def read_wheel(path, expected_hash, target):
    if path.is_symlink() or path.stat().st_size > 128 * 1024 * 1024:
        raise ValueError("Wheel ist Symlink oder zu groß")
    if digest(path.read_bytes()) != expected_hash:
        raise ValueError("Wheel-SHA256 falsch")
    name, version, _, tags = parse_wheel_filename(path.name)
    if not any(tag.interpreter in ("cp313", "py3") and tag.abi in ("cp313", "abi3", "none")
               and tag.platform in target["target_platforms"] for tag in tags):
        raise ValueError("Wheel passt nicht zum CPython-3.13/AArch64-Prüfkandidaten")
    entries, native, notices = {}, [], []
    with zipfile.ZipFile(path) as archive:
        files = [entry for entry in archive.infolist() if not entry.is_dir()]
        names = [entry.filename for entry in files]
        if len(names) != len(set(names)) or len(files) > 20000 or sum(e.file_size for e in files) > 256 * 1024 * 1024:
            raise ValueError("Doppelte Einträge oder Größenlimit im Wheel")
        for entry in files:
            relative = PurePosixPath(entry.filename)
            if relative.is_absolute() or ".." in relative.parts or "\\" in entry.filename or entry.file_size > 64 * 1024 * 1024:
                raise ValueError("Unsicherer oder zu großer Wheel-Eintrag")
            if (entry.external_attr >> 16) & 0o170000 == 0o120000:
                raise ValueError("Keine Symlinks im Wheel")
        record_names = [n for n in names if n.endswith(".dist-info/RECORD")]
        metadata_names = [n for n in names if n.endswith(".dist-info/METADATA")]
        if len(record_names) != 1 or len(metadata_names) != 1:
            raise ValueError("Genau ein RECORD und METADATA erforderlich")
        rows = list(csv.reader(io.StringIO(archive.read(record_names[0]).decode())))
        records = {row[0]: row[1:] for row in rows if len(row) == 3}
        if len(records) != len(rows) or set(records) != set(names):
            raise ValueError("RECORD deckt Wheel nicht genau ab")
        for entry in files:
            data = archive.read(entry)
            checksum, size = records[entry.filename]
            if entry.filename != record_names[0]:
                actual = "sha256=" + base64.urlsafe_b64encode(hashlib.sha256(data).digest()).rstrip(b"=").decode()
                if checksum != actual or size != str(len(data)):
                    raise ValueError("RECORD-Hash/Größe falsch")
            if is_notice(relative := PurePosixPath(entry.filename)):
                target_name = f"notices/{name}/{relative}"
                entries[target_name] = data
                notices.append({"file": target_name, "sha256": digest(data)})
            if entry.filename.endswith(".so") or ".so." in relative.name:
                native.append({"file": entry.filename, "sha256": digest(data), **dynamic_inventory(data)})
        meta = BytesParser().parsebytes(archive.read(metadata_names[0]))
    if canonicalize_name(meta["Name"]) != name or str(version) != meta["Version"]:
        raise ValueError("METADATA und Wheelname widersprechen sich")
    if not SpecifierSet(meta.get("Requires-Python", "")).contains(target["target_environment"]["python_full_version"]):
        raise ValueError("Requires-Python verletzt")
    return {"name": str(name), "version": str(version), "wheel_sha256": expected_hash, "filename": path.name,
            "license_metadata": meta.get("License-Expression") or meta.get("License"),
            "requires_dist": meta.get_all("Requires-Dist", []), "native": native, "notices": notices}, entries


def check_graph(packages, roots, environment):
    if set(environment) != ENVIRONMENT_KEYS or not all(isinstance(value, str) for value in environment.values()):
        raise ValueError("Vollständige explizite Ziel-Markerumgebung erforderlich")
    pending = [("project", Requirement(root)) for root in roots]
    requested, visited, edges, failures = {}, set(), [], []
    while pending:
        parent, requirement = pending.pop(0)
        if parent == "project" and requirement.marker is not None and not requirement.marker.evaluate({**environment, "extra": ""}):
            continue
        name = canonicalize_name(requirement.name)
        if requirement.url:
            failures.append(f"Direkte URL für {name} wird nicht automatisch ausgewertet")
            continue
        package = packages.get(name)
        if package is None or not requirement.specifier.contains(package["version"]):
            failures.append(f"{parent}: Abhängigkeit {name}{requirement.specifier} fehlt oder falsche Version")
            continue
        edges.append({"from": parent, "to": str(name), "specifier": str(requirement.specifier)})
        extras = requested.setdefault(name, set())
        extras.update(requirement.extras)
        state = (name, tuple(sorted(extras)))
        if state in visited:
            continue
        visited.add(state)
        for text in package["requires_dist"]:
            dependency = Requirement(text)
            if dependency.marker is None or any(dependency.marker.evaluate({**environment, "extra": extra})
                                                for extra in {"", *extras}):
                pending.append((str(name), dependency))
    unused = sorted(set(packages) - set(requested))
    return {"closed": not failures and not unused and bool(requested), "edges": edges,
            "failures": failures, "unused_wheels": unused}


def audit(directory, plan):
    if plan.get("format") != 1 or set(plan["target_environment"]) != ENVIRONMENT_KEYS:
        raise ValueError("Ungültiger Prüfplan")
    environment = plan["target_environment"]
    if (any(environment[key] != value for key, value in {"platform_system": "Linux", "sys_platform": "linux",
            "platform_machine": "aarch64", "implementation_name": "cpython", "python_version": "3.13"}.items())
            or not environment["python_full_version"].startswith("3.13.")):
        raise ValueError("Dieses Prüfprofil verlangt Linux/AArch64/CPython 3.13")
    entries, packages = {}, {}
    for wheel in plan["wheels"]:
        filename = wheel["filename"]
        if (Path(filename).name != filename or "\\" in filename or not filename.endswith(".whl")
                or not re.fullmatch(r"[0-9a-f]{64}", wheel["sha256"])):
            raise ValueError("Ungültiger Wheelname oder Sollhash")
        package, notices = read_wheel(Path(directory) / filename, wheel["sha256"], plan)
        if package["name"] in packages:
            raise ValueError("Doppeltes Paket im Prüfplan")
        packages[package["name"]] = package
        entries.update(notices)
    graph = check_graph(packages, plan["roots"], plan["target_environment"])
    report = {"format": 1, "scope": "candidate_base_wheels_only_not_pi_os", "graph": graph,
              "native_graph": native_candidates(packages.values()),
              "target_environment": plan["target_environment"], "packages": list(packages.values()),
              "missing_notice_files": [p["name"] for p in packages.values() if not p["notices"]],
              "hardware_verified": False, "binary_source_mapping_verified": False,
              "license_review_complete": False, "image_approved": False, "ready_for_drive": False}
    return report, entries


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("ordner", type=Path)
    ap.add_argument("--plan", type=Path, default=Path(__file__).resolve().parents[1] / "config/coral-arm64-wheel-audit.json")
    ap.add_argument("--ausgabe", type=Path, required=True)
    ap.add_argument("--belege", type=Path, help="Neues ZIP nur mit originalen Lizenzhinweisen, nicht den Wheels")
    args = ap.parse_args(argv)
    for path in (args.ausgabe, args.belege):
        if path is not None and (path.exists() or path.is_symlink()):
            raise FileExistsError(path)
    report, entries = audit(args.ordner, json.loads(args.plan.read_text()))
    payload = (json.dumps(report, indent=2, ensure_ascii=False) + "\n").encode()
    if args.belege:
        entries["MANIFEST.json"] = payload
        with zipfile.ZipFile(args.belege, "x", compression=zipfile.ZIP_DEFLATED) as archive:
            for name, data in sorted(entries.items()):
                info = zipfile.ZipInfo(name, (2026, 10, 21, 0, 0, 0))
                info.create_system = 3
                info.external_attr = 0o100644 << 16
                info.compress_type = zipfile.ZIP_DEFLATED
                archive.writestr(info, data)
    with args.ausgabe.open("xb") as file:
        file.write(payload)
    print(json.dumps({"graph_closed": report["graph"]["closed"], "packages": len(report["packages"]),
                      "missing_notice_files": report["missing_notice_files"], "hardware_verified": False}))
    return int(not report["graph"]["closed"])


if __name__ == "__main__":
    raise SystemExit(main())
