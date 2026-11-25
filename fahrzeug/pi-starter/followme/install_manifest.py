"""Installationsbestand erfassen/vergleichen, ohne Geräte zu öffnen oder Freigabe.

Keine Zugangsdaten, Hostnamen, pip-URLs oder beliebigen Home-Dateien sammeln.
Gleicher Bestand ist keine Hardware-, Lizenz- oder Sicherheitsabnahme.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib
import json
import platform
import subprocess
from datetime import datetime, timezone
from importlib import metadata
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PACKAGES = ("numpy", "pyserial", "ai-edge-litert", "backports.strenum", "flatbuffers", "ml_dtypes", "packaging",
            "protobuf", "tqdm", "typing_extensions", "pillow", "opencv-python-headless",
            "opencv-contrib-python-headless", "pytest", "ruff", "matplotlib", "pip", "setuptools")
APT_PACKAGES = ("python3-numpy", "python3-opencv", "python3-serial", "python3-picamera2",
                "python3-spidev", "libusb-1.0-0")
MODELS = {
    "ssdlite_mobiledet_coco_qat_postprocess.tflite":
        "32c486140391eb4dc43fca7113ad392be632dc5366687f2731f73d740678693f",
    "ssdlite_mobiledet_coco_qat_postprocess_edgetpu.tflite":
        "b69e508ef2a670e06b80bd3e5559a827d5cd8d557c95d5e332cbf1d31d434a2e",
}
FIRMWARE = {
    "followme_mcu_rpipico.uf2": "ba41ba5020edaa7df4a42203077e6de9678cc014c9bfaaa9d0867589f04c3021",
    "followme_mcu_rpipico2.uf2": "1d16905b16d0f661d85c141efd314c9074d8de83130e58b039ad91b7729938bf",
}


def license_policy():
    """Projektentscheidung, kein automatisches juristisches oder physisches Prüfsiegel."""
    return {"decision_id": "fabian-2026-10-21-mpl2", "scope": "entire_software_stack",
            "mpl_2_0_allowed": True, "additional_paid_license_authorized": False,
            "redistribution_obligations_apply": True, "agpl_allowed": False}


def file_hash(path):
    if not Path(path).is_file():
        return None
    digest = hashlib.sha256()
    with Path(path).open("rb") as file:
        for block in iter(lambda: file.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def package_info(name):
    try:
        dist = metadata.distribution(name)
        # Tatsächlich installierten Python-/Binärcode hashen, ohne ihn zu laden.
        digest = hashlib.sha256()
        count = 0
        for entry in sorted(dist.files or [], key=str):
            if entry.suffix not in (".py", ".so", ".pyd", ".dll", ".dylib") and ".so." not in entry.name:
                continue
            value = file_hash(dist.locate_file(entry))
            if value is not None:
                digest.update((str(entry) + "\0" + value + "\n").encode())
                count += 1
        return {"version": dist.version,
                "license": dist.metadata.get("License-Expression") or dist.metadata.get("License") or None,
                "code_files": count, "code_sha256": digest.hexdigest() if count else None}
    except metadata.PackageNotFoundError:
        return None


def apt_versions():
    if platform.system() != "Linux":
        return None
    try:
        result = subprocess.run(["dpkg-query", "-W", "-f=${binary:Package}\t${Version}\t${db:Status-Status}\n",
                                 *APT_PACKAGES], capture_output=True, text=True, timeout=5, check=False)
    except (OSError, subprocess.TimeoutExpired):
        return None
    versions = {name: None for name in APT_PACKAGES}
    for line in result.stdout.splitlines():
        parts = line.split("\t")
        if len(parts) == 3 and parts[2] == "installed" and parts[0].split(":")[0] in versions:
            versions[parts[0].split(":")[0]] = parts[1]
    return versions


def source_hashes(root):
    paths = [root / "pyproject.toml"]
    for pattern in ("followme/*.py", "tools/*.py", "tools/*.sh", "tools/*.rules", "tools/*.patch",
                    "config/*constraints*.txt", "config/license-sources.json", "config/coral-arm64-wheel-audit.*"):
        paths.extend(root.glob(pattern))
    return {str(p.relative_to(root)): file_hash(p) for p in sorted(set(paths)) if p.is_file()}


def snapshot(profile="base", *, root=ROOT, library=None, image_sha256=None):
    if profile not in ("base", "cpu", "coral"):
        raise ValueError("Unbekanntes Bestandsprofil")
    if image_sha256 is not None and (len(image_sha256) != 64 or any(c not in "0123456789abcdef" for c in image_sha256)):
        raise ValueError("Image-SHA256 muss 64 kleine Hexzeichen haben")
    root = Path(root)
    packages = {name: package_info(name) for name in PACKAGES}
    try:
        cv_version = importlib.import_module("cv2").__version__
    except (ImportError, OSError):
        cv_version = None
    try:
        release = {k: v for k, v in platform.freedesktop_os_release().items()
                   if k in ("ID", "VERSION_ID", "VERSION_CODENAME")}
    except OSError:
        release = {}
    artifacts = {f"firmware/{name}": {"actual": file_hash(root.parent / "firmware/uf2" / name), "expected": expected}
                 for name, expected in FIRMWARE.items()}
    if profile != "base":
        for name, expected in MODELS.items():
            if profile == "coral" or not name.endswith("_edgetpu.tflite"):
                artifacts[f"models/{name}"] = {"actual": file_hash(root / "models" / name), "expected": expected}
    sources = source_hashes(root)
    apt = apt_versions()
    missing = [name for name in ("numpy", "pyserial") if not packages[name]]
    if platform.system() == "Linux" and apt is None:
        missing.append("apt_inventory_unknown")
    if not cv_version:
        missing.append("cv2")
    if profile != "base" and not packages["ai-edge-litert"]:
        missing.append("interpreter_runtime")
    if not sources.get("followme/__init__.py") or not sources.get("pyproject.toml"):
        missing.append("editable_source_tree")
    for name, hashes in artifacts.items():
        if hashes["actual"] != hashes["expected"]:
            missing.append(name)
    library_hash = file_hash(library) if library else None
    if profile == "coral":
        if platform.system() != "Linux" or platform.machine() not in ("aarch64", "arm64"):
            missing.append("linux_arm64_target")
        if not library_hash:
            missing.append("libedgetpu_sha256")
    findings = []
    if packages["tqdm"] and "MPL" in (packages["tqdm"]["license"] or ""):
        findings.append({"package": "tqdm", "version": packages["tqdm"]["version"],
                         "license": packages["tqdm"]["license"], "status": "allowed_by_project_policy",
                         "reason": "MPL-2.0 softwareweit freigegeben; Lizenz-/Quellenhinweise weiter erforderlich"})
    if packages["ai-edge-litert"]:
        findings.append({"component": "native Eigen", "status": "allowed_license_source_review_required",
                         "license": "MPL-2.0 in Upstream-Eigen-Konfiguration",
                         "reason": "Native Eigen-Symbole im Host-Binary; konkrete Quell-/Lizenzzuordnung des Wheels prüfen",
                         "source": "https://github.com/google-ai-edge/LiteRT/blob/v2.3.0/tflite/tools/cmake/modules/eigen.cmake"})
    return {"format": 1, "kind": "install_snapshot", "profile": profile,
            "observed_utc": datetime.now(timezone.utc).isoformat(),
            "identity": {"system": platform.system(), "machine": platform.machine(), "kernel": platform.release(),
                         "os_release": release, "python": platform.python_version(), "packages": packages,
                         "cv2": cv_version, "apt": apt, "sources": sources, "artifacts": artifacts,
                         "library_sha256": library_hash, "base_image_sha256": image_sha256},
            "complete_inventory": not missing, "missing_or_changed": missing,
            "license_policy": license_policy(), "license_findings": findings, "license_review_complete": False,
            "hardware_verified": False, "ready_for_drive": False, "image_approved": False}


def compare(expected, actual):
    identity_keys = {"system", "machine", "kernel", "os_release", "python", "packages", "cv2", "apt",
                     "sources", "artifacts", "library_sha256", "base_image_sha256"}
    for report in (expected, actual):
        if (report.get("format") != 1 or report.get("kind") != "install_snapshot"
                or report.get("profile") not in ("base", "cpu", "coral")
                or type(report.get("complete_inventory")) is not bool
                or not isinstance(report.get("identity"), dict) or set(report["identity"]) != identity_keys
                or not report["identity"].get("sources") or not isinstance(report.get("license_findings"), list)
                or not isinstance(report.get("missing_or_changed"), list)):
            raise ValueError("Ungültiger Installationsbestand")
    differences = [key for key in sorted(set(expected["identity"]) | set(actual["identity"]))
                   if expected["identity"].get(key) != actual["identity"].get(key)]
    if expected["profile"] != actual["profile"]:
        differences.append("profile")
    matches = expected["complete_inventory"] and actual["complete_inventory"] and not differences
    return {"format": 1, "kind": "install_comparison", "matches": matches, "differences": differences,
            "complete_inventory": actual["complete_inventory"], "missing_or_changed": actual["missing_or_changed"],
            "license_policy": license_policy(), "license_findings": actual["license_findings"], "hardware_verified": False,
            "ready_for_drive": False, "image_approved": False}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--profil", choices=("base", "cpu", "coral"), default="base")
    ap.add_argument("--bibliothek", type=Path, help="Nur Datei hashen; niemals laden")
    ap.add_argument("--image-sha256", help="Extern ermittelter Basiskarten-Hash; keine automatische Imageprüfung")
    ap.add_argument("--vergleich", type=Path, help="Vorheriger Master-Bestand, nur lesen")
    ap.add_argument("--ausgabe", type=Path, required=True, help="Neue JSON-Datei")
    args = ap.parse_args(argv)
    with args.ausgabe.open("x", encoding="utf-8") as file:
        actual = snapshot(args.profil, library=args.bibliothek, image_sha256=args.image_sha256)
        report = compare(json.loads(args.vergleich.read_text()), actual) if args.vergleich else actual
        file.write(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False) + "\n")
    print(json.dumps({k: report[k] for k in ("kind", "complete_inventory", "image_approved")}))
    return int(not (report.get("matches") if args.vergleich else report["complete_inventory"]))


if __name__ == "__main__":
    raise SystemExit(main())
