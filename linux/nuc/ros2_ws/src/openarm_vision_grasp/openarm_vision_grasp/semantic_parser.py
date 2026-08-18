"""首版中文桌面抓取指令解析器；只接受明确的“拿起/抓取”意图。"""
from pathlib import Path


PICK_WORDS = ("拿起", "拿住", "抓起", "抓取", "夹起", "捡起")


class ParseError(ValueError):
    pass


def load_config(path):
    import yaml
    with Path(path).expanduser().open(encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def parse_instruction(instruction, config):
    text = str(instruction).strip()
    if not text:
        raise ParseError("指令为空")
    if not any(word in text for word in PICK_WORDS):
        raise ParseError("首版只支持拿起或抓取桌面物品")
    matches = []
    for class_name, profile in config.get("classes", {}).items():
        if not profile.get("enabled", False):
            continue
        aliases = list(profile.get("中文", [])) + [class_name]
        if any(alias and alias in text for alias in aliases):
            matches.append(class_name)
    matches = list(dict.fromkeys(matches))
    if not matches:
        raise ParseError("没有识别出受支持的目标类别")
    # “杯子”可能包含在“马克杯”中，优先保留命中的最长中文别名。
    if len(matches) > 1:
        scored = []
        for class_name in matches:
            aliases = config["classes"][class_name].get("中文", [])
            scored.append((max((len(v) for v in aliases if v in text), default=0), class_name))
        longest = max(v[0] for v in scored)
        matches = [v[1] for v in scored if v[0] == longest]
    if len(matches) != 1:
        raise ParseError("指令同时匹配多个目标类别，请说得更明确")
    return {"action": "pick", "class_name": matches[0], "instruction": text}
