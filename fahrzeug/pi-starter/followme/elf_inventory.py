"""Begrenztes ELF64/AArch64-Inventar ohne Laden, ldd, Shell oder Netzwerk.

Nur DT_NEEDED/SONAME/RPATH/RUNPATH; keine Symbolversions-/Loader-/ABI-Abnahme.
Program Header funktionieren auch ohne Section Header (gestrippte Bibliotheken).
"""

from __future__ import annotations

import struct
from pathlib import PurePosixPath


def dynamic_inventory(data):
    if len(data) < 64 or len(data) > 64 * 1024 * 1024 or data[:7] != b"\x7fELF\x02\x01\x01":
        raise ValueError("ELF64-Little-Endian-Header fehlt, ist abgeschnitten oder zu groß")
    header = struct.unpack_from("<16sHHIQQQIHHHHHH", data)
    _, kind, machine, version, _, phoff, _, _, ehsize, phsize, phcount, *_ = header
    if machine != 183 or kind != 3 or version != 1:
        raise ValueError("ELF ist kein AArch64-Shared-Object")
    if ehsize != 64 or phsize != 56 or not 0 < phcount <= 4096 or phoff < 64 or phoff + phsize * phcount > len(data):
        raise ValueError("ELF-Program-Header ungültig oder nicht unterstütztes erweitertes Format")
    loads, dynamics = [], []
    for i in range(phcount):
        ptype, _, offset, vaddr, _, size, memsize, _ = struct.unpack_from("<IIQQQQQQ", data, phoff + i * phsize)
        if ptype in (1, 2):
            if offset + size > len(data) or size > memsize:
                raise ValueError("ELF-Segment außerhalb der Datei oder größer als Speichersegment")
            (loads if ptype == 1 else dynamics).append((offset, vaddr, size))

    def file_offset(address, size):
        candidates = [offset + address - vaddr for offset, vaddr, length in loads
                      if vaddr <= address and address + size <= vaddr + length]
        if len(candidates) != 1:
            raise ValueError("ELF-Adresse nicht eindeutig in dateigestütztem PT_LOAD")
        return candidates[0]

    if len(dynamics) != 1:
        raise ValueError("Genau ein ELF-PT_DYNAMIC erforderlich")
    offset, address, size = dynamics[0]
    if size == 0 or size % 16 or size > 1024 * 1024 or file_offset(address, size) != offset:
        raise ValueError("ELF-PT_DYNAMIC ungültig")
    tags = {}
    for pos in range(offset, offset + size, 16):
        tag, value = struct.unpack_from("<qQ", data, pos)
        if tag == 0:
            break
        tags.setdefault(tag, []).append(value)
    else:
        raise ValueError("ELF-Dynamiktabelle ohne DT_NULL-Abschluss")
    for tag in (5, 10, 14, 15, 29):
        if len(tags.get(tag, [])) > 1:
            raise ValueError("Doppelter ELF-Einzelwert")
    if 5 not in tags or 10 not in tags or not 0 < tags[10][0] <= 8 * 1024 * 1024:
        raise ValueError("ELF-Dynamik-Stringtabelle fehlt oder ist zu groß")
    length = tags[10][0]
    offset = file_offset(tags[5][0], length)
    strings = data[offset:offset + length]
    if strings[0] != 0 or strings[-1] != 0:
        raise ValueError("ELF-Stringtabelle ohne Null-Begrenzung")
    decoded_bytes = 0

    def name(index):
        nonlocal decoded_bytes
        if index >= len(strings):
            raise ValueError("ELF-Stringindex außerhalb der Tabelle")
        end = strings.find(b"\0", index)
        if end < 0 or end - index > 16384:
            raise ValueError("ELF-String fehlt oder ist zu lang")
        decoded_bytes += end - index
        if decoded_bytes > 256 * 1024:
            raise ValueError("ELF-Textbudget überschritten")
        return strings[index:end].decode("utf-8")

    needed = [name(index) for index in tags.get(1, [])]
    single = {key: name(tags[tag][0]) if tag in tags else None
              for key, tag in (("soname", 14), ("rpath", 15), ("runpath", 29))}
    if any(not text for text in needed) or single["soname"] == "":
        raise ValueError("Leerer ELF-Bibliotheksname")
    return {"elf_class": 64, "elf_machine": machine, "elf_type": kind, "needed": needed, **single}


def native_candidates(packages):
    """Namenskandidaten sind keine Loaderauflösung; Ziel-Systembibliotheken fehlen."""
    objects = [{"package": package["name"], **native} for package in packages for native in package["native"]]
    providers = {}
    for obj in objects:
        for name in {PurePosixPath(obj["file"]).name, obj["soname"]} - {None}:
            providers.setdefault(name, []).append({"package": obj["package"], "file": obj["file"]})
    edges = [{"package": obj["package"], "file": obj["file"], "needed": needed,
              "wheel_name_candidates": sorted(providers.get(needed, []), key=lambda p: (p["package"], p["file"]))}
             for obj in objects for needed in obj["needed"]]
    return {"scope": "Nur direkte ELF-Namen im Wheelbestand, keine Linux-Loaderauflösung",
            "objects": len(objects), "edges": edges,
            "unmatched_needed_names": sorted({edge["needed"] for edge in edges if not edge["wheel_name_candidates"]}),
            "loader_resolution_verified": False, "symbol_versions_checked": False,
            "system_libraries_verified": False, "image_approved": False}
