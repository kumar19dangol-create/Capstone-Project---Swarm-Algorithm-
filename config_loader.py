import json
from pathlib import Path

CONFIG_PATH = Path(__file__).resolve().parent / "config.json"

with CONFIG_PATH.open("r", encoding="utf-8") as file:
    CONFIG = json.load(file)


def cfg(section, key):
    return CONFIG[section][key]
