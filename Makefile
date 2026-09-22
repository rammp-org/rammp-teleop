IMAGE ?= ghcr.io/rammp-org/rammp-teleop
TAG ?= dev
ROS_DOMAIN_ID ?= 0

.PHONY: build run check test smoke lint

build:
	docker build -t $(IMAGE):$(TAG) .

run:
	docker run --rm --network host --ipc host --privileged -v /dev:/dev \
		-e ROS_DOMAIN_ID=$(ROS_DOMAIN_ID) $(IMAGE):$(TAG)

check:
	uv run --with pyyaml python3 scripts/validate_fragment.py \
		rammp-alternative*.yaml

test:
	cd rammp_teleop && uv run --with numpy --with scipy --with pytest python -m pytest test -q
	uv run --with pyyaml --with pytest python -m pytest tests -q

smoke:
	./scripts/smoke.sh $(IMAGE):$(TAG)

lint:
	pre-commit run --all-files
