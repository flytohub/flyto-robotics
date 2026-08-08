# Security

## Reporting a vulnerability

Email **security@flyto2.com**. Please include what you observed, how to
reproduce it, and what you think the impact is. You will get an
acknowledgement; if you do not hear back within a few days, send a reminder
rather than assuming it was received.

Please do not open a public issue for something exploitable, and please do not
test against a robot or an installation that is not yours. Physical machines
move.

## What this repository is

Deterministic mission control for a real robot: a controller that decides
whether to move, a ROS 2 adapter that talks to the hardware, a job runner that
claims work from Flyto2 Cloud, and the operator tools around them. A defect
here can move a machine, so reports about the safety path are the most
interesting ones we receive.

Particularly welcome:

- A sequence where the obstacle guard permits motion it should refuse
- A sensor state that reads as safe when it is unmeasured or stale
- Anything that lets a job move a robot without a valid paired credential
- Anything that makes generated evidence describe a run that did not happen

## The device credential, stated plainly

The robot holds a device secret so it can claim jobs unattended. **It is stored
in clear text** at `~/.flyto/runner-credentials.json`, and reporting that on
its own will be closed as known. Here is the reasoning, so you can aim at what
is actually load-bearing.

The lab robot is a Raspberry Pi 4: no TPM, no secure element. It pairs itself
and must read its own secret at boot with no operator present. Any key it can
use unattended is a key the SD card also holds, so encrypting the credential
against a key stored beside it would look stronger and protect nothing. We
would rather say that than ship the appearance of encryption.

What is actually enforced:

| | |
|---|---|
| File mode | `0600`, set at creation — the file has never existed at any other permission |
| Directory mode | `0700`, and an existing looser one is tightened |
| On read | Refused outright if group or others can read it; the secret is treated as disclosed |
| On write | Atomic rename with `fsync`, so a power cut cannot truncate it into a lost pairing |
| Service | `UMask=0077`, `NoNewPrivileges`, `ProtectSystem=full`, `PrivateTmp` |
| Pairing code | Popped from the environment, never written anywhere |
| Logs | The device **id** is logged. The secret is not, anywhere |

So the boundary is: another account on the robot, a careless backup, a stray
`chmod`, or anything walking the filesystem gets nothing. **Physical possession
of the SD card gets the credential.** That is the honest limit of an unattended
device without hardware key storage, and it is why a lost robot should be
unpaired from Cloud rather than trusted to protect itself.

On a host that *does* have a TPM, the runner reads
`$CREDENTIALS_DIRECTORY` when systemd supplies it — a private tmpfs that never
reaches persistent storage, with the ciphertext at rest sealed to the hardware.
`deploy/systemd/flyto-job-runner.service` documents the provisioning. Where
that is available, this process writes no secret to a filesystem at all.

## Scanning

CodeQL runs on every push and pull request to `main`, plus weekly, with the
`security-extended` query suite. Secret scanning, push protection, non-provider
patterns, validity checks and Dependabot security updates are on.

A quiet scanner is not the same as an absent risk, and this file exists partly
to say where the two differ.
