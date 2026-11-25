"""Lizenz-/Quellenbelege sammeln, nie eine Weitergabe- oder Hardwarefreigabe erteilen.

Ohne --laden vollständig offline. Nur ausgewählte Paketmetadaten/Notices und
hashgebundene Quellen sammeln, keine Home-Dateien, direkten pip-URLs oder Geräte.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import urllib.request
import zipfile
from importlib import metadata
from pathlib import Path, PurePosixPath
from urllib.parse import urlsplit

from .install_manifest import PACKAGES, ROOT, license_policy

MAX_ASSET_BYTES = 128 * 1024 * 1024
MAX_NOTICE_BYTES = 2 * 1024 * 1024
HOSTS = {"raw.githubusercontent.com", "codeload.github.com", "files.pythonhosted.org", "storage.googleapis.com"}
PROJECT_FILES = ("LIZENZEN.md", "LIZENZPAKET.md", "SOFTWARE-TODO.md", "ARM64-PAKETPLAN.md", "config/license-sources.json",
                 "config/core-constraints.txt", "config/cpu-host-constraints.txt", "config/coral-arm64-wheel-audit.json",
                 "config/coral-arm64-wheel-audit.txt", "tools/coral_bauen.sh",
                 "tools/coral-trixie.patch", "tools/coral_modelle.py")
OPEN_ITEMS = (
    "Eigen-/Drittkomponenten dem konkreten veröffentlichten LiteRT-Wheel zuordnen; Referenzquellen sind kein Binärnachweis.",
    "Vollständigen Python-Paketgraph einschließlich aktiver Extras und weiterer nativer Wheel-Komponenten prüfen.",
    "Debian-/Pi-OS-Pakete samt Copyright, passenden Quellen und libusb/LGPL-Material des Zielimages ergänzen.",
    "Pico-Core/SDK und Firmware-Bibliotheken samt Quellen, Lizenz-/Änderungshinweisen zuordnen.",
    "Eigenen Projektcode separat lizenzieren lassen; MPL-Abhängigkeitsfreigabe entscheidet das nicht.",
    "Konkreten Image-/Wheel-/Buildbestand mit Belegen abgleichen und Weitergabepflichten manuell prüfen.",
)


def digest(data):
    return hashlib.sha256(data).hexdigest()


def check_url(url):
    parsed = urlsplit(url)
    if (parsed.scheme != "https" or parsed.netloc not in HOSTS or parsed.query or parsed.fragment
            or parsed.username or parsed.password):
        raise ValueError("Quell-URL muss auf einem erlaubten HTTPS-Quellhost liegen")


class SourceRedirectHandler(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        check_url(newurl)  # Vor dem Folge-Request prüfen, nicht erst nach dessen Antwort.
        return super().redirect_request(req, fp, code, msg, headers, newurl)


def open_source(url):
    return urllib.request.build_opener(SourceRedirectHandler()).open(url, timeout=30)


def load_registry(path):
    registry = json.loads(Path(path).read_text(encoding="utf-8"))
    if registry.get("format") != 1 or not isinstance(registry.get("assets"), list):
        raise ValueError("Ungültiges Quellenregister")
    names = set()
    for asset in registry["assets"]:
        name = asset["filename"]
        if (not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]*", name) or ".." in name or name in names
                or not re.fullmatch(r"[0-9a-f]{64}", asset["sha256"])
                or type(asset["size_bytes"]) is not int or not 0 < asset["size_bytes"] <= MAX_ASSET_BYTES):
            raise ValueError("Ungültiger Name, Quellhash, Umfang oder doppelter Registereintrag")
        check_url(asset["url"])
        names.add(name)
    return registry


def read_checked(path, *, limit):
    if path.is_symlink():
        raise ValueError("Keine symbolischen Links in Lizenzbelegen")
    with path.open("rb") as file:
        data = file.read(limit + 1)
    if len(data) > limit:
        raise ValueError("Lizenzbeleg überschreitet Größenlimit")
    return data


def verify_asset(asset, data):
    if len(data) != asset["size_bytes"] or digest(data) != asset["sha256"]:
        raise ValueError(f"Quellgröße/SHA256 falsch: {asset['filename']}")


def fetch_assets(registry, cache):
    """Nur explizit laden, vorhandenen Cache prüfen, niemals Archive entpacken/ausführen."""
    cache = Path(cache)
    if cache.is_symlink():
        raise ValueError("Cache darf kein symbolischer Link sein")
    cache.mkdir(parents=True, exist_ok=True)
    for asset in registry["assets"]:
        path = cache / asset["filename"]
        if path.exists() or path.is_symlink():
            verify_asset(asset, read_checked(path, limit=asset["size_bytes"]))
            continue
        check_url(asset["url"])
        with open_source(asset["url"]) as response:
            check_url(response.geturl())
            data = response.read(asset["size_bytes"] + 1)
        verify_asset(asset, data)
        with path.open("xb") as file:
            file.write(data)


def is_notice(path):
    name = path.name.upper()
    return any(word in name for word in ("LICENSE", "LICENCE", "NOTICE", "COPYING", "COPYRIGHT", "AUTHORS"))


def collect_package(name, entries):
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]*", name) or ".." in name:
        raise ValueError("Ungültiger Paketname")
    try:
        dist = metadata.distribution(name)
    except metadata.PackageNotFoundError:
        return {"name": name, "installed": False, "notices": []}
    result = {"name": name, "installed": True, "version": dist.version,
              "license_metadata": dist.metadata.get("License-Expression") or dist.metadata.get("License"),
              "notices": [], "missing_notices": []}
    # Keine vollständigen METADATA/direct_url.json kopieren: können lokale/private URLs enthalten.
    for entry in sorted(dist.files or [], key=str):
        relative = PurePosixPath(str(entry))
        if not is_notice(relative):
            continue
        if relative.is_absolute() or ".." in relative.parts or "\\" in str(relative):
            raise ValueError("Unsicherer Paket-Lizenzpfad")
        path = Path(dist.locate_file(entry))
        if path.is_symlink() or not path.resolve().is_relative_to(Path(dist.locate_file("")).resolve()):
            raise ValueError("Paket-Lizenzpfad verlässt die Distribution oder ist ein symbolischer Link")
        if not path.is_file():
            result["missing_notices"].append(str(relative))
            continue
        data = read_checked(path, limit=MAX_NOTICE_BYTES)
        target = f"packages/{name}/{relative}"
        entries[target] = data
        result["notices"].append({"file": target, "sha256": digest(data)})
    return result


def collect(*, root=ROOT, cache=None, packages=PACKAGES):
    root = Path(root)
    registry = load_registry(root / "config/license-sources.json")
    if cache is not None and Path(cache).is_symlink():
        raise ValueError("Cache darf kein symbolischer Link sein")
    entries = {}
    package_reports = [collect_package(name, entries) for name in packages]
    source_reports = []
    for asset in registry["assets"]:
        item = dict(asset, collected=False, binary_source_mapping_verified=False)
        package = next((p for p in package_reports if p["name"] == asset["component"] and p["installed"]), None)
        item["installed_version"] = package["version"] if package else None
        item["matches_installed_version"] = package["version"] == asset["version"] if package else None
        path = Path(cache) / asset["filename"] if cache is not None else None
        if path is not None and (path.exists() or path.is_symlink()):
            data = read_checked(path, limit=asset["size_bytes"])
            verify_asset(asset, data)
            item.update(collected=True, file=f"upstream/{asset['filename']}")
            entries[item["file"]] = data
        source_reports.append(item)
    missing_project_files = []
    for name in PROJECT_FILES:
        path = root / name
        if not path.is_file():
            missing_project_files.append(name)
            continue
        entries[f"project/{name}"] = read_checked(path, limit=MAX_NOTICE_BYTES)
    report = {"format": 1, "kind": "license_evidence_bundle", "scope": "selected_host_packages_and_build_references",
              "license_policy": license_policy(), "packages": package_reports, "sources": source_reports,
              "unarchived_source_locations": registry.get("unarchived_source_locations", []),
              "packages_without_notice_files": [p["name"] for p in package_reports if p["installed"] and not p["notices"]],
              "missing_project_files": missing_project_files, "open_items": list(OPEN_ITEMS),
              "license_review_complete": False, "redistribution_ready": False, "hardware_verified": False,
              "image_approved": False, "ready_for_drive": False}
    entries["MANIFEST.json"] = (json.dumps(report, indent=2, ensure_ascii=False, sort_keys=True) + "\n").encode()
    lines = ["# Lizenz-/Quellenbelege — Arbeitsstand, keine Weitergabefreigabe", "",
             "Nur ausgewählte installierte Pakete und vorbereitete Build-Referenzen, kein vollständiges Pi-SBOM.",
             "Alle Freigabefelder bleiben false. Quellenzuordnung eines konkreten Binaries ist nicht bewiesen.", "",
             "## Installierte Pakete", ""]
    for package in package_reports:
        if package["installed"]:
            lines.append(f"- {package['name']} {package['version']}: {len(package['notices'])} Hinweisdateien")
    lines.extend(["", "## Noch offen", "", *(f"- {item}" for item in OPEN_ITEMS)])
    entries["LESEN.md"] = ("\n".join(lines) + "\n").encode()
    entries["SHA256SUMS"] = "".join(f"{digest(data)}  {name}\n" for name, data in sorted(entries.items())).encode()
    return report, entries


def build_archive(output, *, root=ROOT, cache=None, packages=PACKAGES):
    output = Path(output)
    if output.exists() or output.is_symlink():
        raise FileExistsError(output)
    report, entries = collect(root=root, cache=cache, packages=packages)
    with zipfile.ZipFile(output, "x", compression=zipfile.ZIP_DEFLATED) as archive:
        for name, data in sorted(entries.items()):
            info = zipfile.ZipInfo(name, (2026, 10, 21, 0, 0, 0))
            info.create_system = 3
            info.external_attr = 0o100644 << 16
            info.compress_type = zipfile.ZIP_DEFLATED
            archive.writestr(info, data)
    return report, digest(output.read_bytes())


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--ausgabe", type=Path, required=True, help="Neues ZIP; vorhandene Datei bleibt erhalten")
    ap.add_argument("--cache", type=Path, help="Nur bekannte hashgebundene Quelldateien lesen")
    ap.add_argument("--laden", action="store_true", help="Explizit Netzwerkzugriff zum Füllen des Quellen-Caches")
    args = ap.parse_args(argv)
    if args.ausgabe.exists() or args.ausgabe.is_symlink():
        raise FileExistsError(args.ausgabe)
    if args.laden:
        if args.cache is None:
            ap.error("--laden braucht --cache")
        fetch_assets(load_registry(ROOT / "config/license-sources.json"), args.cache)
    report, checksum = build_archive(args.ausgabe, cache=args.cache)
    print(json.dumps({"sha256": checksum, "sources_collected": sum(s["collected"] for s in report["sources"]),
                      "installed_packages": sum(p["installed"] for p in report["packages"]),
                      "redistribution_ready": False, "license_review_complete": False}))
    return 0  # Erfolgreiche Sammlung ist ausdrücklich keine erfolgreich abgeschlossene Lizenzprüfung.


if __name__ == "__main__":
    raise SystemExit(main())
