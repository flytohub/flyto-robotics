"""Stable reason/action codes for the product lifecycle and health surfaces.

A support call starts with a code, not a stack trace. Every code here is part of
the customer-facing contract: the string is what the runbook indexes, what the
recovery portal renders, and what a support bundle carries. Codes are added, not
renamed -- renaming one silently invalidates every runbook entry that quotes it.

Each reason maps to exactly one action so that an operator is never told what is
wrong without being told what to do about it.
"""

from __future__ import annotations

__all__ = ["ACTIONS", "REASONS", "action_for", "describe", "is_known"]

# reason code -> (human meaning, action code)
REASONS: dict[str, tuple[str, str]] = {
    "ok": ("the requested lifecycle operation completed", "none"),
    "no_change": ("the requested state was already in place", "none"),
    "dry_run": ("nothing was written; this was a plan", "rerun_without_dry_run"),
    "release_exists_with_different_content": (
        "an immutable release directory already holds different bytes",
        "choose_a_new_version",
    ),
    "release_missing": ("the requested release is not installed", "list_releases"),
    "release_payload_invalid": ("the release payload failed its content check", "reobtain_release"),
    "unit_validation_failed": (
        "a systemd unit would not behave as written and was not installed",
        "inspect_report_defects",
    ),
    "post_switch_health_failed": (
        "the new release failed its health check and was rolled back",
        "collect_support_bundle",
    ),
    "no_rollback_target": ("there is no earlier known-good release to return to", "reinstall"),
    "post_switch_readiness_failed": (
        "the new release started but is not usable, and was rolled back",
        "collect_support_bundle",
    ),
    "activation_snapshot_invalid": (
        "the immutable record of how a release was activated is missing or altered",
        "install_that_version_explicitly",
    ),
    "activation_not_recorded": (
        "this device has no record of ever activating that release",
        "install_that_version_explicitly",
    ),
    "activation_not_reproducible": (
        "that release's unit set no longer renders as it did when it was activated",
        "install_that_version_explicitly",
    ),
    "not_installed": ("no release has ever been made current", "run_install"),
    "prefix_not_writable": ("the install prefix cannot be written", "rerun_with_privileges"),
    "identity_missing": ("the device identity file is absent", "provision_identity"),
    "config_unreadable": ("the persistent configuration could not be read", "restore_config"),
    "current_symlink_foreign": (
        "current exists but is not a symlink this installer manages",
        "move_aside_and_reinstall",
    ),
    "operation_in_progress": (
        "another lifecycle operation holds the device lock",
        "retry_after_current_operation",
    ),
    "systemctl_failed": (
        "a systemctl step failed; the change was undone",
        "collect_support_bundle",
    ),
    "service_not_active": (
        "the service was started but is not running; the change was undone",
        "collect_support_bundle",
    ),
    "rollback_failed": (
        "an operation failed and the automatic undo did not complete",
        "escalate_to_support",
    ),
    "state_drift": (
        "the active release and the recorded state disagree",
        "rerun_install_to_reconcile",
    ),
    "install_failed": (
        "the install did not complete; the change was undone",
        "collect_support_bundle",
    ),
    "profiles_invalid": ("the unit profile registry is missing or malformed", "reobtain_release"),
    "systemd_required": (
        "a real install was asked to skip the service lifecycle",
        "rerun_without_no_systemd",
    ),
    "note_rejected": ("the support note is outside the note policy", "shorten_note_to_reference"),
    "io_failed": ("a file could not be read or written", "check_disk_and_permissions"),
    "unexpected_error": (
        "an unhandled condition was reported as a code, not a crash",
        "collect_support_bundle",
    ),
}

ACTIONS: dict[str, str] = {
    "none": "no operator action required",
    "rerun_without_dry_run": "re-run the same command without --dry-run",
    "choose_a_new_version": "publish under a new version; releases are immutable",
    "list_releases": "run `flyto-robot lifecycle status` to list installed releases",
    "reobtain_release": "re-download the release payload and verify its digest",
    "inspect_report_defects": "read report.defects[]; each names section, key, and reason",
    "collect_support_bundle": "run `flyto-robot support-bundle` and attach it to the ticket",
    "reinstall": "run a fresh install of a known-good version",
    "install_that_version_explicitly": (
        "run `flyto-robot install --version <V>` for that release; a one-command "
        "rollback only reproduces an activation this device actually recorded"
    ),
    "run_install": "run the installer once to create the first release",
    "rerun_with_privileges": "re-run with the privileges the prefix requires",
    "provision_identity": "provision the device identity before starting services",
    "restore_config": "restore /etc config from backup; releases never write it",
    "move_aside_and_reinstall": "move the foreign `current` entry aside, then reinstall",
    "retry_after_current_operation": "wait for the running operation to finish, then retry",
    "escalate_to_support": (
        "stop here: attach `flyto-robot support-bundle` and do not run further "
        "lifecycle commands; the device is between two known states"
    ),
    "rerun_without_no_systemd": (
        "drop --no-systemd, or add --dry-run to rehearse; an install that skips "
        "systemd leaves a device that will not come back after a reboot"
    ),
    "shorten_note_to_reference": (
        "replace the note with a ticket reference; bundles carry no free text"
    ),
    "check_disk_and_permissions": "check free space and the privileges the path requires",
    "rerun_install_to_reconcile": (
        "re-run `flyto-robot install` for the release `current` points at; the "
        "device was interrupted between activation and its state write"
    ),
}


def is_known(reason: str) -> bool:
    return reason in REASONS


def action_for(reason: str) -> str:
    """Return the action code for ``reason``.

    Unknown reasons fail closed to a bundle collection rather than to "none":
    telling an operator that an unrecognised failure needs no action is worse
    than telling them to collect evidence.
    """

    if reason not in REASONS:
        return "collect_support_bundle"
    return REASONS[reason][1]


def describe(reason: str) -> str:
    if reason not in REASONS:
        return "unrecognised reason code"
    return REASONS[reason][0]
