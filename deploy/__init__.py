"""Side-effect-free launchers for Flyto2 device deployment."""

from __future__ import annotations

import argparse
import json
import sys


def job_runner_main() -> int:
    """Handle launcher help, then invoke the canonical runner unchanged."""
    # argparse normally repeats an unexpected positional value in its error.
    # In pair mode that value is likely a mistakenly supplied secret, so reject
    # it before parsing and emit no user-controlled bytes.
    if len(sys.argv) > 2 and sys.argv[1] == "pair" and sys.argv[2:] != ["--help"]:
        print(
            json.dumps(
                {
                    "action_code": "use_pairing_code_environment",
                    "ok": False,
                    "reason": "pairing_argument_refused",
                },
                sort_keys=True,
                separators=(",", ":"),
            )
        )
        return 2
    parser = argparse.ArgumentParser(
        prog="flyto-job-runner",
        description="Claim Flyto2 device jobs and execute supported work safely.",
    )
    subparsers = parser.add_subparsers(dest="command")
    subparsers.add_parser(
        "pair",
        help="pair this installation using FLYTO_PAIRING_CODE",
        description="Pair this installation using FLYTO_PAIRING_CODE.",
    )
    arguments = parser.parse_args()

    from . import flyto_job_runner

    if arguments.command == "pair":
        return flyto_job_runner.pair_main()
    return flyto_job_runner.main()
