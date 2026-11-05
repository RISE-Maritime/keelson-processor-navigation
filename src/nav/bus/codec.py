"""Encode and decode payloads for the bus, as keelson protobuf or as a raw_json rider.

A payload is always a dict in protobuf's JSON mapping (snake_case field names, enum names as
strings, RFC 3339 timestamps, unset fields omitted). That one representation is what makes the
rider-to-proto swap a codec flag rather than a rewrite: the rider carries exactly the dict that
``json_format.ParseDict`` turns into the proto once keelson registers the type.

Rider wire format, on subject ``raw_json`` as ``keelson.TimestampedString``::

    {"record": "<subject>", "rider_version": 1, "payload": {...}}
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass

from .riders import RIDER_VERSION, rider_subjects

RAW_JSON = "raw_json"


class CodecError(ValueError):
    """A subject or payload the codec cannot handle."""


@dataclass(frozen=True)
class Decoded:
    subject: str
    payload: dict
    enclosed_at_ns: int


def _proto_class(subject: str):
    import keelson

    if not keelson.is_subject_well_known(subject):
        raise CodecError(f"subject {subject!r} is not registered in the installed keelson SDK")
    return keelson.get_protobuf_message_class_from_type_name(keelson.get_subject_schema(subject))


def has_fields(subject: str, *names: str) -> bool:
    """Whether the installed SDK's type for `subject` has every one of these fields: what a
    payload may carry without ParseDict refusing it on an older build."""
    try:
        fields = _proto_class(subject).DESCRIPTOR.fields_by_name
    except (CodecError, ImportError, KeyError):
        return False
    return all(n in fields for n in names)


def _to_proto_bytes(subject: str, payload: dict) -> bytes:
    from google.protobuf import json_format

    try:
        message = json_format.ParseDict(payload, _proto_class(subject)())
    except json_format.ParseError as exc:
        raise CodecError(f"payload does not fit {subject!r}: {exc}") from exc
    return message.SerializeToString()


def _from_proto_bytes(subject: str, data: bytes) -> dict:
    from google.protobuf import json_format

    message = _proto_class(subject).FromString(data)
    return json_format.MessageToDict(message, preserving_proto_field_name=True)


class Codec:
    """One codec per process. ``riders`` are the subjects routed onto raw_json in rider mode."""

    def __init__(self, mode: str, riders: frozenset[str] | None = None):
        if mode not in ("rider", "proto"):
            raise CodecError(f"unknown codec mode {mode!r}")
        self.mode = mode
        self.riders = frozenset() if mode == "proto" else (riders or rider_subjects())

    def wire_subject(self, subject: str) -> str:
        return RAW_JSON if subject in self.riders else subject

    def encode(self, subject: str, payload: dict, *, enclosed_at_ns: int | None = None) -> bytes:
        import keelson

        now = enclosed_at_ns or time.time_ns()
        if subject in self.riders:
            body = json.dumps(
                {"record": subject, "rider_version": RIDER_VERSION, "payload": payload},
                sort_keys=True,
                separators=(",", ":"),
            )
            inner = _to_proto_bytes(RAW_JSON, {"timestamp": _rfc3339(now), "value": body})
        else:
            inner = _to_proto_bytes(subject, payload)
        return keelson.enclose(inner, enclosed_at=now)

    def decode(self, wire_subject: str, data: bytes) -> Decoded:
        import keelson

        _, enclosed_at, inner = keelson.uncover(data)
        if wire_subject != RAW_JSON:
            return Decoded(wire_subject, _from_proto_bytes(wire_subject, inner), enclosed_at)
        text = _from_proto_bytes(RAW_JSON, inner).get("value", "")
        try:
            body = json.loads(text)
        except json.JSONDecodeError as exc:
            raise CodecError(f"raw_json value is not JSON: {exc}") from exc
        if not isinstance(body, dict) or "record" not in body:
            raise CodecError("raw_json value is not a rider: no 'record' field")
        if body.get("rider_version") != RIDER_VERSION:
            raise CodecError(f"unsupported rider_version {body.get('rider_version')!r}")
        return Decoded(body["record"], body.get("payload", {}), enclosed_at)


def _rfc3339(ns: int) -> str:
    from google.protobuf.timestamp_pb2 import Timestamp

    ts = Timestamp()
    ts.FromNanoseconds(ns)
    return ts.ToJsonString()


def rfc3339(ns: int) -> str:
    """A nanosecond epoch time in the protobuf JSON Timestamp form."""
    return _rfc3339(ns)
