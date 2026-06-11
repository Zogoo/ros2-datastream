# Quality gates. Python tooling runs inside the ROS image so no host installs
# are needed; FE tooling uses the local node_modules (npm install once).

ROS_IMAGE ?= onsen-ros:latest
PY_PKGS    = src/onsen_ai_worker src/onsen_robot_state src/onsen_dummy_robot
DOCKER_PY  = docker run --rm \
	-v $(PWD)/src:/ws/src \
	-v $(PWD)/shared:/ros2_ws/shared \
	-v $(PWD)/pyproject.toml:/ws/pyproject.toml \
	-w /ws --entrypoint bash $(ROS_IMAGE) -lc

.PHONY: check check-py check-fe lint-py type-py test-py lint-fe test-fe build e2e

check: check-py check-fe
	@echo "✓ all quality gates passed"

check-py: lint-py type-py test-py

lint-py:
	$(DOCKER_PY) "python3 -m ruff check $(PY_PKGS)"

type-py:
	$(DOCKER_PY) "python3 -m mypy $(PY_PKGS)"

test-py:
	$(DOCKER_PY) "PYTHONPATH=/ws/src/onsen_ai_worker:/ws/src/onsen_robot_state:/ws/src/onsen_dummy_robot \
		python3 -m pytest $(addsuffix /test,$(PY_PKGS)) -q"

check-fe: lint-fe test-fe

lint-fe:
	cd frontend && npx eslint src tests

test-fe:
	cd frontend && npx vitest run

build:
	docker compose build

e2e:
	docker compose --profile e2e up --abort-on-container-exit --exit-code-from e2e
