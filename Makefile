PYTHON ?= python3

.PHONY: ai-dry-run ai4all-medication-showcase ai4all-showcase assets benchmark-robot-mcp careflow-dry-run dry-run facility-contract gazebo-lab gazebo-matrix gazebo-shortcut gazebo-video lab-contract lint nav2-closed-loop nav2-stress ros2-execution-grant ros2-pairing soak test verify

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

benchmark-robot-mcp:
	$(PYTHON) scripts/benchmark_robot_mcp.py \
		--cases 101 \
		--output-dir output/benchmarks

lab-contract:
	$(PYTHON) -m flyto_robotics.cli validate-lab-scenario \
		scenarios/gazebo/careflow-adversarial.json

facility-contract:
	$(PYTHON) -m flyto_robotics.resource_binding \
		examples/resource-plans/gazebo-shortcut-forward-30cm.json \
		--workflow shortcut.forward.30cm.v1 \
		--resource flyto-rover-sim-001 \
		--capability mobility.move_relative \
		--adapter robotics.gazebo \
		--space gazebo-lab

ros2-pairing:
	$(PYTHON) -m flyto_robotics.cli verify-ros2-pairing \
		--manifest examples/ros2-adapters/flyto2-standard.json \
		--runtime examples/ros2-runtime/ready-sim.json \
		--at 2026-08-01T10:00:00Z

ros2-execution-grant:
	$(PYTHON) -m flyto_robotics.cli authorize-ros2-execution \
		--manifest examples/ros2-adapters/flyto2-standard.json \
		--runtime examples/ros2-runtime/ready-sim.json \
		--resource-plan examples/resource-plans/nav2-hospital-delivery.json \
		--workflow hospital_delivery.v1 \
		--resource flyto-rover-sim-001 \
		--capability robotics.motion.navigate@1 \
		--space gazebo-nav2-lab \
		--at 2026-08-01T10:00:00Z

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

gazebo-shortcut:
	./scripts/run-shortcut-gazebo.sh

gazebo-video:
	FLYTO_ROBOTICS_RECORD_VIDEO=1 ./scripts/run-gazebo-lab.sh

nav2-closed-loop:
	./scripts/run_nav2_closed_loop.sh

nav2-stress:
	./scripts/run_nav2_stress.sh

ai4all-showcase:
	./scripts/run-ai4all-showcase.sh

ai4all-medication-showcase:
	./scripts/run-ai4all-medication-showcase.sh

verify: lint test assets dry-run ai-dry-run careflow-dry-run lab-contract facility-contract ros2-pairing ros2-execution-grant
