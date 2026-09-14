"""A small reader for the draft .proto files: message blocks, their fields, nested messages, enums.

Enough to check that a payload emits only names a draft declares. Not a protobuf parser.
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field
from pathlib import Path

DRAFTS = Path(__file__).resolve().parent.parent / "docs" / "upstream"

_FIELD = re.compile(r"^\s*(?:optional |repeated )?[\w.]+\s+(\w+)\s*=\s*\d+;")
_ENUM_VALUE = re.compile(r"^\s*([A-Z][A-Z0-9_]+)\s*=\s*\d+;")


@dataclass
class Message:
    name: str
    fields: set[str] = field(default_factory=set)
    messages: dict[str, Message] = field(default_factory=dict)
    enum_values: set[str] = field(default_factory=set)
    enums: dict[str, set[str]] = field(default_factory=dict)
    field_types: dict[str, str] = field(default_factory=dict)


def _strip_comments(text: str) -> str:
    return "\n".join(line.split("//", 1)[0] for line in text.splitlines())


def parse(filename: str, message: str) -> Message:
    text = _strip_comments((DRAFTS / filename).read_text())
    lines = text.splitlines()
    root = Message("<file>")
    stack: list[tuple[str, Message | None]] = [("file", root)]
    for line in lines:
        m = re.match(r"^\s*message\s+(\w+)\s*\{", line)
        e = re.match(r"^\s*enum\s+(\w+)\s*\{", line)
        o = re.match(r"^\s*oneof\s+(\w+)\s*\{", line)
        if m:
            msg = Message(m.group(1))
            parent = next(p for kind, p in reversed(stack) if kind in ("message", "file"))
            parent.messages[msg.name] = msg
            stack.append(("message", msg))
            continue
        if e:
            owner = next(p for kind, p in reversed(stack) if kind in ("message", "file"))
            owner.enums[e.group(1)] = set()
            stack.append(("enum", e.group(1)))
            continue
        if o:
            stack.append(("oneof", None))
            continue
        if line.strip() == "}":
            stack.pop()
            continue
        kind = stack[-1][0]
        owner = next(p for k, p in reversed(stack) if k in ("message", "file"))
        if kind == "enum":
            ev = _ENUM_VALUE.match(line)
            if ev:
                owner.enum_values.add(ev.group(1))
                owner.enums[stack[-1][1]].add(ev.group(1))
        else:
            f = _FIELD.match(line)
            if f:
                words = line.split()
                type_word = words[1] if words[0] in ("optional", "repeated") else words[0]
                owner.fields.add(f.group(1))
                owner.field_types[f.group(1)] = type_word
    return root.messages[message]


def check_payload(payload: dict, msg: Message, scope: list[Message] | None = None) -> list[str]:
    """Names in ``payload`` the draft does not declare, and values of enum-typed fields the enum
    does not declare. Only fields whose declared type is an enum are value-checked, so a plain
    string that happens to be upper case (a context name, a base64 digest) is not flagged."""
    scope = [msg, *(scope or [])]
    problems = []
    for key, value in payload.items():
        if key not in msg.fields:
            problems.append(f"{msg.name}.{key}")
            continue
        type_name = msg.field_types.get(key, "").split(".")[-1]
        nested = next((s.messages[type_name] for s in scope if type_name in s.messages), None)
        enum = next((s.enums[type_name] for s in scope if type_name in s.enums), None)
        values = value if isinstance(value, list) else [value]
        for v in values:
            if isinstance(v, dict) and nested is not None:
                problems += check_payload(v, nested, scope)
            elif enum is not None and v not in enum:
                problems.append(f"{msg.name}.{key}={v}")
    return problems


def _all_enum_values(msg: Message) -> set[str]:
    out = set(msg.enum_values)
    for child in msg.messages.values():
        out |= _all_enum_values(child)
    return out
