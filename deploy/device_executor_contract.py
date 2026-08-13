"""Validation for the installed device-executor protocol.

This module deliberately has no dependencies on an application package.  It
describes data exchanged at the executor boundary; it does not discover or run
executors.
"""

from __future__ import annotations

import json
import math
import os
import re
from collections.abc import Mapping, Set
from typing import Any

CONTRACT_VERSION = "device-executor-v1"

MAX_JSON_BYTES = 65_536
MAX_JSON_DEPTH = 12
MAX_JSON_NODES = 2_048
MAX_LIST_ITEMS = 256
MAX_OBJECT_ITEMS = 256
MAX_STRING_LENGTH = 8_192
MAX_KEY_LENGTH = 128
MAX_IDENTIFIER_LENGTH = 128
MAX_DETAIL_LENGTH = 1_024
MAX_ARGV_ITEMS = 32
MAX_ARG_LENGTH = 1_024
MAX_MODULE_IDS = 256
MAX_EVIDENCE_ITEMS = 32
MIN_TIMEOUT_SECONDS = 1
MAX_TIMEOUT_SECONDS = 300

_IDENTIFIER = re.compile(r"[A-Za-z0-9][A-Za-z0-9._-]*\Z")
_ENTRY_POINT = re.compile(
    r"[A-Za-z_][A-Za-z0-9_]*(?:\.[A-Za-z_][A-Za-z0-9_]*)*"
    r":[A-Za-z_][A-Za-z0-9_]*(?:\.[A-Za-z_][A-Za-z0-9_]*)*\Z"
)


class ContractValidationError(ValueError):
    """A content-free validation failure safe to expose across boundaries."""

    def __init__(self, reason_code: str) -> None:
        self.reason_code = reason_code
        super().__init__(reason_code)


def _fail(reason_code: str) -> None:
    raise ContractValidationError(reason_code)


def _exact_object(
    value: Any,
    required: Set[str],
    optional: Set[str] = frozenset(),
) -> Mapping[str, Any]:
    if not isinstance(value, dict):
        _fail("object_required")
    keys = set(value)
    if keys != required | (keys & optional):
        _fail("fields_invalid")
    return value


def _identifier(value: Any, reason: str = "identifier_invalid") -> str:
    if (not isinstance(value, str) or len(value) > MAX_IDENTIFIER_LENGTH
            or not _IDENTIFIER.fullmatch(value)):
        _fail(reason)
    return value


def _bounded_text(value: Any, maximum: int, reason: str) -> str:
    if (not isinstance(value, str) or not value or len(value) > maximum
            or any(ord(char) < 32 or ord(char) == 127 for char in value)):
        _fail(reason)
    return value


def validate_json(value: Any) -> Any:
    """Validate bounded JSON and return a detached tree without coercion."""

    nodes = 0
    active: set[int] = set()

    def visit(item: Any, depth: int) -> Any:
        nonlocal nodes
        nodes += 1
        if nodes > MAX_JSON_NODES:
            _fail("json_nodes_exceeded")
        if depth > MAX_JSON_DEPTH:
            _fail("json_depth_exceeded")
        if item is None or isinstance(item, (str, bool, int, float)):
            if isinstance(item, str) and len(item) > MAX_STRING_LENGTH:
                _fail("json_string_exceeded")
            if isinstance(item, float) and not math.isfinite(item):
                _fail("json_non_finite")
            return item
        if not isinstance(item, (list, dict)):
            _fail("json_type_invalid")
        identity = id(item)
        if identity in active:
            _fail("json_cycle")
        active.add(identity)
        try:
            if isinstance(item, list):
                if len(item) > MAX_LIST_ITEMS:
                    _fail("json_list_exceeded")
                return [visit(child, depth + 1) for child in item]
            if len(item) > MAX_OBJECT_ITEMS:
                _fail("json_object_exceeded")
            result = {}
            for key, child in item.items():
                if not isinstance(key, str):
                    _fail("json_key_invalid")
                if not key or len(key) > MAX_KEY_LENGTH or any(
                        ord(char) < 32 or ord(char) == 127 for char in key):
                    _fail("json_key_invalid")
                result[key] = visit(child, depth + 1)
            return result
        finally:
            active.remove(identity)

    detached = visit(value, 0)
    try:
        encoded = json.dumps(
            detached, ensure_ascii=False, allow_nan=False, separators=(",", ":"),
        ).encode("utf-8")
    except (TypeError, ValueError, UnicodeEncodeError):
        _fail("json_encoding_invalid")
    if len(encoded) > MAX_JSON_BYTES:
        _fail("json_bytes_exceeded")
    return detached


def validate_request(value: Any) -> dict[str, Any]:
    obj = _exact_object(value, {"contract_version", "module_id", "params"})
    if obj["contract_version"] != CONTRACT_VERSION:
        _fail("contract_version_unsupported")
    return {"contract_version": CONTRACT_VERSION,
            "module_id": _identifier(obj["module_id"], "module_id_invalid"),
            "params": validate_json(obj["params"])}


def validate_prepared_payload(value: Any) -> Any:
    """Validate an executor's opaque, JSON-compatible prepared payload."""

    return validate_json(value)


def validate_source(value: Any) -> dict[str, str]:
    obj = _exact_object(value, {"provider", "source_id"})
    return {"provider": _identifier(obj["provider"], "source_invalid"),
            "source_id": _identifier(obj["source_id"], "source_invalid")}


def validate_evidence(value: Any) -> dict[str, Any]:
    obj = _exact_object(value, {"kind", "usable"}, {"detail", "zone", "source"})
    if type(obj["usable"]) is not bool:
        _fail("evidence_usable_invalid")
    result: dict[str, Any] = {
        "kind": _identifier(obj["kind"], "evidence_kind_invalid"),
        "usable": obj["usable"],
    }
    if "detail" in obj:
        result["detail"] = _bounded_text(
            obj["detail"], MAX_DETAIL_LENGTH, "evidence_detail_invalid"
        )
    if "zone" in obj:
        result["zone"] = _identifier(obj["zone"], "evidence_zone_invalid")
    if "source" in obj:
        result["source"] = validate_source(obj["source"])
    return result


def validate_result(value: Any) -> dict[str, Any]:
    obj = _exact_object(
        value, {"contract_version", "status", "reason_code", "evidence"}, {"detail"},
    )
    if obj["contract_version"] != CONTRACT_VERSION:
        _fail("contract_version_unsupported")
    status = obj["status"]
    if status not in {"succeeded", "failed", "refused"}:
        _fail("result_status_invalid")
    reason_code = _identifier(obj["reason_code"], "reason_code_invalid")
    evidence = obj["evidence"]
    if not isinstance(evidence, list) or len(evidence) > MAX_EVIDENCE_ITEMS:
        _fail("result_evidence_invalid")
    if status != "succeeded" and evidence:
        _fail("result_evidence_forbidden")
    result: dict[str, Any] = {
        "contract_version": CONTRACT_VERSION,
        "status": status,
        "reason_code": reason_code,
        "evidence": [validate_evidence(item) for item in evidence],
    }
    if "detail" in obj:
        result["detail"] = _bounded_text(obj["detail"], MAX_DETAIL_LENGTH, "result_detail_invalid")
    return result


def validate_executor_manifest(value: Any) -> dict[str, Any]:
    if not isinstance(value, dict):
        _fail("object_required")
    transport = value.get("transport")
    common = {"contract_version", "provider", "module_ids", "transport"}
    if transport == "python_entry_point":
        obj = _exact_object(value, common | {"entry_point"})
    elif transport == "json_stdio":
        obj = _exact_object(value, common | {"command", "timeout_seconds"})
    else:
        _fail("transport_unsupported")
    if obj["contract_version"] != CONTRACT_VERSION:
        _fail("contract_version_unsupported")
    provider = _identifier(obj["provider"], "provider_invalid")
    module_ids = obj["module_ids"]
    if (not isinstance(module_ids, list) or not module_ids
            or len(module_ids) > MAX_MODULE_IDS):
        _fail("module_ids_invalid")
    normalized_ids = [_identifier(item, "module_id_invalid") for item in module_ids]
    if len(set(normalized_ids)) != len(normalized_ids):
        _fail("module_ids_duplicate")
    result: dict[str, Any] = {"contract_version": CONTRACT_VERSION,
                              "provider": provider, "module_ids": normalized_ids,
                              "transport": transport}
    if transport == "python_entry_point":
        entry_point = obj["entry_point"]
        if (not isinstance(entry_point, str) or len(entry_point) > MAX_IDENTIFIER_LENGTH
                or not _ENTRY_POINT.fullmatch(entry_point)):
            _fail("entry_point_invalid")
        result["entry_point"] = entry_point
        return result
    command = obj["command"]
    if (not isinstance(command, list) or not command
            or len(command) > MAX_ARGV_ITEMS):
        _fail("command_invalid")
    normalized_command = [
        _bounded_text(arg, MAX_ARG_LENGTH, "command_invalid") for arg in command
    ]
    executable = normalized_command[0]
    if not os.path.isabs(executable):
        _fail("command_not_absolute")
    if os.path.normpath(executable) != executable:
        _fail("command_not_normalized")
    timeout = obj["timeout_seconds"]
    if (type(timeout) is not int or not MIN_TIMEOUT_SECONDS <= timeout <= MAX_TIMEOUT_SECONDS):
        _fail("timeout_invalid")
    result.update({"command": normalized_command, "timeout_seconds": timeout})
    return result


__all__ = [
    "CONTRACT_VERSION", "ContractValidationError", "validate_evidence",
    "validate_executor_manifest", "validate_json", "validate_prepared_payload",
    "validate_request", "validate_result", "validate_source",
]
