VENV    := .venv
PYTHON  := $(VENV)/bin/python
MAP     ?= maps/easy/01_linear_path.txt
MAIN    := fly_in.py
SOURCES := $(wildcard *.py)
CACHES  := __pycache__ .mypy_cache .pytest_cache

MYPY_FLAGS = --warn-return-any --warn-unused-ignores \
             --ignore-missing-imports --disallow-untyped-defs \
             --check-untyped-defs

.PHONY: install run debug lint lint-strict clean fclean

# Every target below uses the python of the virtual environment, so there is
# nothing to activate by hand. If the venv is missing, it is created first.
install:
	python3 -m venv $(VENV)
	$(PYTHON) -m pip install --upgrade pip
	$(PYTHON) -m pip install -r requirements.txt flake8 mypy
	@echo "Ready. To use the venv in your own shell: source $(VENV)/bin/activate"

$(PYTHON):
	$(MAKE) install

run: $(PYTHON)
	$(PYTHON) $(MAIN) $(MAP)

debug: $(PYTHON)
	$(PYTHON) -m pdb $(MAIN) $(MAP)

lint: $(PYTHON)
	$(PYTHON) -m flake8 $(SOURCES)
	$(PYTHON) -m mypy $(SOURCES) $(MYPY_FLAGS)

lint-strict: $(PYTHON)
	$(PYTHON) -m flake8 $(SOURCES)
	$(PYTHON) -m mypy $(SOURCES) --strict

clean:
	rm -rf $(CACHES)
	find . -name "__pycache__" -type d -prune -exec rm -rf {} +

fclean: clean
	rm -rf $(VENV)
