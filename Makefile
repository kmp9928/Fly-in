TO_RUN := main.py
VENV := .venv
PYTHON := $(VENV)/bin/python
PIP := $(VENV)/bin/pip
INCLUDE_FILES := \
				errors.py \
				graph.py \
				input_parser.py \
				main.py \
				min_cost_max_flow.py \
				models.py \
				renderer.py \
				route_planner.py \
				simulation_engine.py

.PHONY: help venv install run lint lint-strict clean debug

help:
	@echo "Commands:"
	@echo "make install			Installs development dependencies: mypy, flake8, pip(upgrade) and pydantic"
	@echo "make run 			runs main.py"
	@echo "make debug			runs the main script in pdb"
	@echo "make lint			runs flake8 and mypy tests"
	@echo "make lint-strict		runs flake8 and mypy --strict"
	@echo "make clean			cleans pycache mypy_cache"

install:
ifeq ("$(wildcard $(VENV))","")
	@echo "Virtual environment not found. Creating $(VENV)..."
	@python3 -c "import sys; sys.exit(0) if sys.version_info >= (3,10) \
	else sys.exit(1)" || { echo "Error: Python 3.10+ required"; exit 1; }
	@python3 -m venv $(VENV)
else
	@echo "Virtual environment already exists."
endif
	@$(PYTHON) -m pip install --upgrade pip
	@$(PYTHON) -m pip install -r requirements.txt
	@echo "Dependencies installed"

$(VENV):
	@make install

run: $(VENV)
	@$(PYTHON) $(TO_RUN)


debug:
	@$(PYTHON) -m pdb $(TO_RUN)


lint:
	@$(PYTHON) -m flake8 $(INCLUDE_FILES)
	@$(PYTHON) -m mypy \
		--warn-return-any \
		--warn-unused-ignores \
		--ignore-missing-imports \
		--disallow-untyped-defs \
		--check-untyped-defs $(INCLUDE_FILES)


lint-strict:
	@$(PYTHON) -m flake8 $(INCLUDE_FILES)
	@$(PYTHON) -m mypy --strict $(INCLUDE_FILES)


clean:
	@rm -rf __pycache__ .mypy_cache
	@echo "Cleaned build artifacts and cache files"
