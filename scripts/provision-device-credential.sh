#!/usr/bin/env bash
# Provision a device credential for an offline / enterprise installation.
#
#   sudo scripts/provision-device-credential.sh [--output PATH] [--check]
#
# Reads a credential JSON on stdin, encrypts it with systemd-creds, and writes
# it where the enterprise drop-in expects it. Verifies the round trip before
# reporting success, because a credential that cannot be decrypted at boot is a
# robot that will not start and a site visit to find out why.
#
# The plaintext never reaches a disk and never appears in a command line: it
# arrives on stdin and is piped straight to systemd-creds. Anything on argv is
# visible in `ps` to every user on the machine, which is why the secret is not
# an argument.
#
# There is no pairing here on purpose. An offline installation has no Cloud to
# pair against; an administrator issues the credential out of band and brings
# it to the machine.

set -euo pipefail

OUTPUT=/etc/flyto/device.cred
CREDENTIAL_NAME=flyto-device
CHECK_ONLY=0

while [ $# -gt 0 ]; do
  case "$1" in
    --output) OUTPUT="${2:?--output needs a path}"; shift 2 ;;
    --check)  CHECK_ONLY=1; shift ;;
    -h|--help) sed -n '2,20p' "$0"; exit 0 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done

die() { echo "provision: $*" >&2; exit 1; }

command -v systemd-creds >/dev/null || die "systemd-creds is not installed; systemd 250 or newer is required"

if [ "$CHECK_ONLY" = 1 ]; then
  [ -f "$OUTPUT" ] || die "$OUTPUT does not exist"
  systemd-creds decrypt --name="$CREDENTIAL_NAME" "$OUTPUT" - >/dev/null \
    || die "$OUTPUT exists but cannot be decrypted on this host. If it was
        encrypted against another machine's TPM, that is expected — re-run
        provisioning here."
  echo "ok: $OUTPUT decrypts on this host"
  exit 0
fi

[ "$(id -u)" -eq 0 ] || die "must run as root: encryption uses the system key or TPM"

# What the key is bound to, reported rather than assumed. An operator who
# believes they have TPM sealing when they do not has the wrong threat model.
if [ -e /dev/tpmrm0 ] || [ -e /dev/tpm0 ]; then
  BINDING="TPM (a copy of this disk will not decrypt on another host)"
else
  BINDING="host key in /var/lib/systemd (root-only, but on this same disk —
        this is NOT protection against someone holding the drive)"
fi

[ -t 0 ] && die "expects the credential JSON on stdin, e.g.
        sudo $0 < device.json
        Do not pass the secret as an argument; argv is visible in ps."

PLAINTEXT="$(cat)"

# Validate before encrypting: a malformed credential encrypts perfectly well
# and fails at boot, far from here.
printf '%s' "$PLAINTEXT" | python3 -c '
import json, sys
try:
    data = json.load(sys.stdin)
except json.JSONDecodeError as exc:
    sys.exit(f"not valid JSON: {exc}")
if not isinstance(data, dict):
    sys.exit("expected a JSON object")
missing = [k for k in ("device_id", "device_secret") if not data.get(k)]
if missing:
    sys.exit("missing or empty: " + ", ".join(missing))
if set(data) - {"device_id", "device_secret"}:
    extra = ", ".join(sorted(set(data) - {"device_id", "device_secret"}))
    sys.exit(f"unexpected keys ({extra}); ship only what the runner reads")
' || die "the credential on stdin was rejected; nothing was written"

OUTPUT_DIR="$(dirname "$OUTPUT")"
if [ ! -d "$OUTPUT_DIR" ]; then
  install -d -m 0700 "$OUTPUT_DIR"
else
  # Never re-permission a directory this script did not create. An earlier
  # version ran `install -d -m 0700` unconditionally, so --output under /tmp
  # turned /tmp itself into 0700 root and locked every other user out of it.
  # It did that to a live robot. Creating is ours to do; tightening someone
  # else's directory is not.
  case "$(stat -c %a "$OUTPUT_DIR")" in
    700|750|755|500|550|555) : ;;
    *) echo "provision: warning: $OUTPUT_DIR is mode $(stat -c %a "$OUTPUT_DIR"); " \
            "the credential file itself will still be 0600" >&2 ;;
  esac
fi

TEMP="$(mktemp "${OUTPUT}.XXXXXX")"
trap 'rm -f "$TEMP"' EXIT
chmod 0600 "$TEMP"

printf '%s' "$PLAINTEXT" | systemd-creds encrypt --name="$CREDENTIAL_NAME" - "$TEMP" \
  || die "systemd-creds encrypt failed; nothing was written"

# Prove it comes back before replacing anything. Compare through a hash so the
# secret is not printed even on failure.
BEFORE="$(printf '%s' "$PLAINTEXT" | sha256sum | cut -d' ' -f1)"
AFTER="$(systemd-creds decrypt --name="$CREDENTIAL_NAME" "$TEMP" - | sha256sum | cut -d' ' -f1)"
[ "$BEFORE" = "$AFTER" ] || die "the encrypted credential did not decrypt back to what was given"

mv "$TEMP" "$OUTPUT"
chmod 0600 "$OUTPUT"
trap - EXIT

cat <<SUMMARY
provisioned: $OUTPUT (0600, root)
sealed to:   $BINDING

Next:
  install -D -m 0644 deploy/systemd/flyto-job-runner.service.d/enterprise-credential.conf \\
      /etc/systemd/system/flyto-job-runner.service.d/enterprise-credential.conf
  systemctl daemon-reload && systemctl restart flyto-job-runner.service

Then confirm the runner took the systemd credential rather than falling back:
  journalctl -u flyto-job-runner.service -n 20
  test ! -e /var/lib/flyto-runner/runner-credentials.json && echo "no secret on disk"
SUMMARY
