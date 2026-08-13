import json
import math
from pathlib import Path

import pytest

from deploy.device_executor_contract import (
    CONTRACT_VERSION,
    MAX_ARG_LENGTH,
    MAX_ARGV_ITEMS,
    MAX_EVIDENCE_ITEMS,
    MAX_IDENTIFIER_LENGTH,
    MAX_JSON_BYTES,
    MAX_JSON_DEPTH,
    MAX_LIST_ITEMS,
    MAX_MODULE_IDS,
    MAX_OBJECT_ITEMS,
    MAX_STRING_LENGTH,
    MAX_TIMEOUT_SECONDS,
    MIN_TIMEOUT_SECONDS,
    ContractValidationError,
    validate_evidence,
    validate_executor_manifest,
    validate_json,
    validate_prepared_payload,
    validate_request,
    validate_result,
    validate_source,
)


def manifest(**changes):
    value = {"contract_version": CONTRACT_VERSION, "provider": "future.provider",
             "module_ids": ["generic.module"], "transport": "python_entry_point",
             "entry_point": "future_executor.plugin:Executor"}
    value.update(changes)
    return value


def test_valid_python_entry_point_and_json_stdio_manifests():
    assert validate_executor_manifest(manifest())["provider"] == "future.provider"
    stdio = manifest(transport="json_stdio", command=["/opt/flyto/executor", "--json"],
                     timeout_seconds=20)
    stdio.pop("entry_point")
    assert validate_executor_manifest(stdio)["command"] == ["/opt/flyto/executor", "--json"]


@pytest.mark.parametrize("change,reason", [
    ({"command": ["relative"]}, "command_not_absolute"),
    ({"command": ["/opt/executors/../executor"]}, "command_not_normalized"),
    ({"command": ["/opt//executor"]}, "command_not_normalized"),
    ({"command": ["/bin/tool\nsecret"]}, "command_invalid"),
])
def test_stdio_command_is_absolute_bounded_argv_not_shell_text(change, reason):
    value = manifest(transport="json_stdio", timeout_seconds=5, **change)
    value.pop("entry_point")
    with pytest.raises(ContractValidationError) as caught:
        validate_executor_manifest(value)
    assert caught.value.reason_code == reason


def test_transport_conditional_fields_are_exact_and_job_has_no_command():
    with pytest.raises(ContractValidationError, match="fields_invalid"):
        validate_executor_manifest(manifest(command=["/bin/tool"]))
    with pytest.raises(ContractValidationError, match="fields_invalid"):
        validate_request({"contract_version": CONTRACT_VERSION, "module_id": "x",
                          "params": {}, "command": ["/bin/tool"]})

    stdio = manifest(transport="json_stdio", command=["/bin/tool"],
                     timeout_seconds=1, shell=True)
    stdio.pop("entry_point")
    with pytest.raises(ContractValidationError, match="fields_invalid"):
        validate_executor_manifest(stdio)


def test_manifest_transport_fields_remain_strictly_conditional():
    with pytest.raises(ContractValidationError, match="fields_invalid"):
        validate_executor_manifest(manifest(timeout_seconds=1))
    stdio = manifest(transport="json_stdio", command=["/bin/tool"],
                     timeout_seconds=1)
    with pytest.raises(ContractValidationError, match="fields_invalid"):
        validate_executor_manifest(stdio)


def test_json_validation_controls_sizes_depth_nodes_and_non_finite():
    with pytest.raises(ContractValidationError, match="json_key_invalid"):
        validate_json({"bad\nkey": 1})
    nested = 0
    for _ in range(MAX_JSON_DEPTH + 1):
        nested = [nested]
    with pytest.raises(ContractValidationError, match="json_depth_exceeded"):
        validate_json(nested)
    node_tree = None
    for _ in range(11):
        node_tree = [node_tree, node_tree]
    with pytest.raises(ContractValidationError, match="json_nodes_exceeded"):
        validate_json(node_tree)
    for number in (math.nan, math.inf, -math.inf):
        with pytest.raises(ContractValidationError, match="json_non_finite"):
            validate_json(number)


def test_json_result_has_no_mutation_or_aliasing_with_caller():
    shared = {"items": [1]}
    source = {"left": shared, "right": shared}
    result = validate_json(source)
    result["left"]["items"].append(2)
    assert source == {"left": shared, "right": shared}
    assert result["right"] == {"items": [1]}


def test_json_utf8_serialized_byte_boundary():
    exact = ["a" + "é" * 8_191, "é" * 8_191, "é" * 8_191, "é" * 8_188]
    assert len(json.dumps(
        exact, ensure_ascii=False, separators=(",", ":")
    ).encode("utf-8")) == MAX_JSON_BYTES
    assert validate_json(exact) == exact
    over = [*exact[:-1], exact[-1] + "a"]
    with pytest.raises(ContractValidationError, match="json_bytes_exceeded"):
        validate_json(over)


@pytest.mark.parametrize("value,reason", [
    ("x" * MAX_STRING_LENGTH, None),
    ("x" * (MAX_STRING_LENGTH + 1), "json_string_exceeded"),
    ({str(index): index for index in range(MAX_OBJECT_ITEMS)}, None),
    ({str(index): index for index in range(MAX_OBJECT_ITEMS + 1)},
     "json_object_exceeded"),
    (list(range(MAX_LIST_ITEMS)), None),
    (list(range(MAX_LIST_ITEMS + 1)), "json_list_exceeded"),
])
def test_json_item_boundaries(value, reason):
    if reason is None:
        assert validate_json(value) == value
    else:
        with pytest.raises(ContractValidationError, match=reason):
            validate_json(value)


def test_json_rejects_non_string_keys_and_cycles():
    with pytest.raises(ContractValidationError, match="json_key_invalid"):
        validate_json({1: "value"})
    cyclic = []
    cyclic.append(cyclic)
    with pytest.raises(ContractValidationError, match="json_cycle"):
        validate_json(cyclic)


def test_request_prepared_and_result_outputs_are_detached():
    params = {"nested": [1]}
    request = validate_request({"contract_version": CONTRACT_VERSION,
                                "module_id": "generic.module", "params": params})
    prepared = validate_prepared_payload(params)
    evidence = {"kind": "observation", "usable": True,
                "source": {"provider": "future", "source_id": "one"}}
    result = validate_result({"contract_version": CONTRACT_VERSION,
                              "status": "succeeded", "reason_code": "ok",
                              "evidence": [evidence]})
    params["nested"].append(2)
    evidence["source"]["source_id"] = "changed"
    assert request["params"] == {"nested": [1]}
    assert prepared == {"nested": [1]}
    assert result["evidence"][0]["source"]["source_id"] == "one"


def test_result_status_and_evidence_rules():
    base = {"contract_version": CONTRACT_VERSION, "status": "failed",
            "reason_code": "executor.failed", "evidence": []}
    assert validate_result(base) == base
    with pytest.raises(ContractValidationError, match="result_evidence_forbidden"):
        validate_result({**base, "evidence": [{"kind": "observation", "usable": True}]})
    assert validate_result({**base, "status": "succeeded", "evidence": []})["evidence"] == []


def test_result_evidence_accepts_exactly_32_and_rejects_33():
    item = {"kind": "observation", "usable": True}
    base = {"contract_version": CONTRACT_VERSION, "status": "succeeded",
            "reason_code": "ok"}
    assert len(validate_result({**base, "evidence": [item] * MAX_EVIDENCE_ITEMS})[
        "evidence"
    ]) == 32
    with pytest.raises(ContractValidationError, match="result_evidence_invalid"):
        validate_result({**base, "evidence": [item] * (MAX_EVIDENCE_ITEMS + 1)})


@pytest.mark.parametrize("status", ["failed", "refused"])
def test_non_success_results_cannot_carry_evidence(status):
    with pytest.raises(ContractValidationError, match="result_evidence_forbidden"):
        validate_result({"contract_version": CONTRACT_VERSION, "status": status,
                         "reason_code": "not.available",
                         "evidence": [{"kind": "observation", "usable": False}]})


def test_manifest_argv_timeout_and_module_id_boundaries():
    def stdio(**changes):
        value = manifest(transport="json_stdio", command=["/bin/tool"],
                         timeout_seconds=MIN_TIMEOUT_SECONDS)
        value.pop("entry_point")
        value.update(changes)
        return value

    assert len(validate_executor_manifest(stdio(
        command=["/bin/tool"] + ["x"] * (MAX_ARGV_ITEMS - 1)
    ))["command"]) == MAX_ARGV_ITEMS
    with pytest.raises(ContractValidationError, match="command_invalid"):
        validate_executor_manifest(stdio(command=["/bin/tool"] +
                                         ["x"] * MAX_ARGV_ITEMS))
    assert validate_executor_manifest(stdio(command=["/" + "x" *
                                                      (MAX_ARG_LENGTH - 1)]))
    with pytest.raises(ContractValidationError, match="command_invalid"):
        validate_executor_manifest(stdio(command=["/" + "x" * MAX_ARG_LENGTH]))
    for timeout in (MIN_TIMEOUT_SECONDS, MAX_TIMEOUT_SECONDS):
        assert validate_executor_manifest(stdio(timeout_seconds=timeout))[
            "timeout_seconds"
        ] == timeout
    for timeout in (MIN_TIMEOUT_SECONDS - 1, MAX_TIMEOUT_SECONDS + 1):
        with pytest.raises(ContractValidationError, match="timeout_invalid"):
            validate_executor_manifest(stdio(timeout_seconds=timeout))

    module_ids = [f"module.{index}" for index in range(MAX_MODULE_IDS)]
    assert validate_executor_manifest(manifest(module_ids=module_ids))["module_ids"] == module_ids
    with pytest.raises(ContractValidationError, match="module_ids_invalid"):
        validate_executor_manifest(manifest(module_ids=module_ids + ["one.more"]))
    with pytest.raises(ContractValidationError, match="module_ids_duplicate"):
        validate_executor_manifest(manifest(module_ids=["same", "same"]))
    with pytest.raises(ContractValidationError, match="module_id_invalid"):
        validate_executor_manifest(manifest(module_ids=["x" *
                                                         (MAX_IDENTIFIER_LENGTH + 1)]))


@pytest.mark.parametrize("validator,value", [
    (validate_executor_manifest, manifest(extra=True)),
    (validate_result, {"contract_version": CONTRACT_VERSION, "status": "succeeded",
                       "reason_code": "ok", "evidence": [], "extra": True}),
    (validate_evidence, {"kind": "observation", "usable": True, "extra": True}),
    (validate_source, {"provider": "future", "source_id": "one", "extra": True}),
])
def test_contract_objects_reject_unknown_keys(validator, value):
    with pytest.raises(ContractValidationError, match="fields_invalid"):
        validator(value)


@pytest.mark.parametrize("provider,source_id", [
    ("ros_image", "overhead-1"), ("avfoundation", "usb-1"),
    ("future.provider", "source.alpha"),
])
def test_producer_shaped_evidence_and_open_source_vocabulary(provider, source_id):
    evidence = {"kind": "zone.overview", "zone": "ward-a", "usable": True,
                "detail": "frame_fresh",
                "source": {"provider": provider, "source_id": source_id}}
    assert validate_evidence(evidence) == evidence


@pytest.mark.parametrize("field", [
    "pixels", "image", "frame", "device", "device_id", "serial", "topic", "url", "output",
])
def test_evidence_rejects_privacy_and_arbitrary_output_fields(field):
    with pytest.raises(ContractValidationError, match="fields_invalid"):
        validate_evidence({"kind": "observation", "usable": False, field: "secret"})


def test_validation_error_never_echoes_payload_content():
    secret = "super-secret-value"
    with pytest.raises(ContractValidationError) as caught:
        validate_request(
            {"contract_version": CONTRACT_VERSION, "module_id": secret + "/", "params": {}}
        )
    assert secret not in str(caught.value)


def test_contract_has_no_domain_hard_coding_or_runtime_execution():
    text = Path("deploy/device_executor_contract.py").read_text()
    assert "flyto_robotics" not in text
    assert "subprocess" not in text
    assert "ros_image" not in text
    assert "avfoundation" not in text
