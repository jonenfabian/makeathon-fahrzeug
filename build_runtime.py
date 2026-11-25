"""Nur Orga/CI: plattformeigenen Firmware-Host bauen und Prüfsummen ablegen."""
import hashlib
import importlib.util
import json
import shutil
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
HOST = ROOT / "fahrzeug/firmware/host_sim"
sys.path.insert(0, str(ROOT / "fahrzeug/pi-starter"))
from followme.sim_binary import platform_key, source_hash

spec = importlib.util.spec_from_file_location("native_build", HOST / "build.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
binary = module.build()
folder = HOST / "prebuilt" / platform_key()
folder.mkdir(parents=True, exist_ok=True)
shutil.copy2(binary, folder / binary.name)
manifest = {"platform": platform_key(), "sources_sha256": source_hash(HOST.parent),
            "binary_sha256": hashlib.sha256(binary.read_bytes()).hexdigest()}
(folder / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
shutil.copytree(folder, ROOT / "runtime-built" / platform_key(), dirs_exist_ok=True)
print(f"Simulator fertig: {platform_key()}")
