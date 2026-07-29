PYTHON ?= python3

.PHONY: ai-dry-run assets careflow-dry-run dry-run gazebo-lab gazebo-matrix gazebo-video lab-contract lint soak test verify

lint:
	$(PYTHON) -m ruff check .

test:
	$(PYTHON) -m pytest

assets:
	$(PYTHON) -m flyto_robotics.cli validate-assets

dry-run:
	$(PYTHON) -m flyto_robotics.cli dry-run examples/jobs/pharmacy-to-ward.json

ai-dry-run:
	$(PYTHON) -m flyto_robotics.cli dry-run-plan \
		--job examples/jobs/pharmacy-to-ward.json \
		--plan examples/plans/blue-yellow-purple.json

careflow-dry-run:
	$(PYTHON) -m flyto_robotics.cli dry-run-plan \
		--job examples/jobs/pharmacy-to-ward.json \
		--plan examples/plans/careflow-human-gate.json

lab-contract:
	$(PYTHON) -m flyto_robotics.cli validate-lab-scenario \
		scenarios/gazebo/careflow-adversarial.json

soak:
	$(PYTHON) -m flyto_robotics.cli soak-plan \
		--job examples/jobs/pharmacy-to-ward.json \
		--plan examples/plans/careflow-human-gate.json \
		--runs 50 \
		--output-dir results/deterministic-soak

gazebo-lab:
	./scripts/run-gazebo-lab.sh

gazebo-matrix:
	./scripts/run-gazebo-matrix.sh

gazebo-video:
	FLYTO_ROBOTICS_RECORD_VIDEO=1 ./scripts/run-gazebo-lab.sh

verify: lint test assets dry-run ai-dry-run careflow-dry-run lab-contract
