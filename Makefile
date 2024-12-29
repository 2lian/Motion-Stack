# Variables
SYMLINK_INSTALL := 1  # Set to 0 to disable symlink-install

PYTHON = python3
DOCS_DIR = docs
BUILD_DIR = $(DOCS_DIR)/build
SRC_DOC = $(DOCS_DIR)/source
API_DIR = $(SRC_DOC)/api
INSTALL_SETUP = install/setup.sh
README_FILE = README.md

MOTION_STACK_DOC = $(API_DIR)/motion_stack/.timestamp
EASY_DOC = $(API_DIR)/easy_robot_control/.timestamp
API_DOC = $(API_DIR)/.timestamp
MD_DOC = $(DOCS_DIR)/build/markdown/.timestamp
HTML_DOC = $(DOCS_DIR)/build/html/.timestamp

ROS2_DISTRO := $(shell (test -d /opt/ros/humble && echo humble) || (test -d /opt/ros/foxy && echo foxy) || (echo unknown))

INI_FILES := $(shell find src -type f ! -path "*cache*" ! -path "*/.*" ! -name "*cache*" -name "*__init__.py")
INI_DIRS := $(sort $(dir $(INI_FILES)))
PY_FILES := $(foreach dir, $(INI_DIRS), $(wildcard $(dir)*.py))
SRC_FILESaaa := $(shell find src -type f ! -path "*cache*" ! -path "*/.*" ! -name "*cache*")

# Conditional symlink-install flag
ifeq ($(SYMLINK_INSTALL), 1)
  SYMLINK_FLAG := --symlink-install
  SRC_FILES := $(filter-out $(PY_FILES), $(SRC_FILESaaa))
else
  SYMLINK_FLAG :=
  SRC_FILES := $(SRC_FILESaaa)
endif


.PHONY: hello
hello:
	echo SRC_FILES

.PHONY: all
all: README.md

.PHONY: clean
clean:
	- rm -r build/ log/ install/
	- rm src/*/requirements*.txt
	- rm -r src/*.egg-info/
	- rm -r $(DOCS_DIR)/build
	- rm -r $(API_DIR)

$(INSTALL_SETUP): $(SRC_FILES)
	@echo "Building workspace..."
	@. /opt/ros/$(ROS2_DISTRO)/setup.sh && \
	RCUTILS_COLORIZED_OUTPUT=1 && \
	colcon build $(SYMLINK_FLAG) --cmake-args -Wno-dev

$(EASY_DOC): $(INSTALL_SETUP) $(shell find src/easy_robot_control -type f ! -path "*cache*" ! -path "*/.*" ! -name "*cache*")
	@echo "Generating easy_robot_control auto-documentation..."
	@mkdir -p $(API_DIR)/easy_robot_control
	. $(INSTALL_SETUP) && \
	@sphinx-apidoc -M -d 2 -o $(API_DIR)/easy_robot_control src/easy_robot_control/easy_robot_control
	@touch $(EASY_DOC)

$(MOTION_STACK_DOC): $(INSTALL_SETUP) $(shell find src/motion_stack -type f ! -path "*cache*" ! -path "*/.*" ! -name "*cache*")
	@echo "Generating motion_stack auto-documentation..."
	@mkdir -p $(API_DIR)/motion_stack
	. $(INSTALL_SETUP) && \
	@sphinx-apidoc -M -d 2 -o $(API_DIR)/motion_stack src/motion_stack/motion_stack
	@touch $(MOTION_STACK_DOC)

$(API_DOC): $(EASY_DOC) $(MOTION_STACK_DOC)
	@echo "Generating docstring python-documentation..."
	mkdir -p $(SRC_DOC)/_static
	@touch $(API_DOC)

$(MD_DOC): $(API_DOC) $(wildcard $(SRC_DOC)/manual/*) $(SRC_DOC)/conf.py $(SRC_DOC)/index.rst
	@echo "Generating markdown documentation..."
	@sphinx-build -M markdown $(SRC_DOC) $(DOCS_DIR)/build
	@touch $(MD_DOC)

README.md: $(MD_DOC)
	@echo "Generating README.md ..."
	@cp $(DOCS_DIR)/build/markdown/index.md README.md
	@prefix="$(DOCS_DIR)/build/markdown/" && \
	sed -i "s|\[\([^]]*\)\](\([^)]*\.md.*\))|[\1]($$prefix\2)|g" "README.md"
	@sed -i '1s|^|<!-- This file is auto-generated from the docs. refere to ./docs/source/manual/README.rst -->\n|' README.md
	@newline="Clone, then open the full html documentation in your browser : \`./docs/build/html/index.html\`" && \
	sed -i "/^# Guides:$$/a $$newline" README.md
	@newline="\ " && \
	sed -i "/^# Guides:$$/a $$newline" README.md

$(HTML_DOC): $(API_DOC) $(wildcard $(SRC_DOC)/*)
	@echo "Generating html documentation..."
	@sphinx-build -M html $(SRC_DOC) $(DOCS_DIR)/build
	@touch $(HTML_DOC)

.PHONY: md_doc
md_doc: $(MD_DOC) README.md

.PHONY: html_doc
html_doc: $(HTML_DOC)

.PHONY: full_doc
full_doc: html_doc md_doc

.PHONY: build
build: $(INSTALL_SETUP)

check-pip:

src/easy_robot_control/requirements.txt: src/easy_robot_control/setup.py
	@echo "Building Python requirements..."
	@which pip3 > /dev/null || (echo "pip3 not found, installing..." && \
	sudo apt update && sudo apt install -y python3-pip)
	pip install pip-tools
	pip-compile -o src/easy_robot_control/requirements.txt src/easy_robot_control/setup.py

src/easy_robot_control/requirements-dev.txt: src/easy_robot_control/setup.py
	@echo "Building Python-dev requirements..."
	pip-compile  --extra dev -o src/easy_robot_control/requirements-dev.txt src/easy_robot_control/setup.py

src/easy_robot_control/easy_robot_control.egg-info/.dev-stamp: src/easy_robot_control/requirements-dev.txt
	@echo "Installing Python dependencies with pip"
	CXXFLAGS="-fno-fat-lto-objects --param ggc-min-expand=10 --param ggc-min-heapsize=2048" pip install -r src/easy_robot_control/requirements-dev.txt --force-reinstall --upgrade
	@touch src/easy_robot_control/easy_robot_control.egg-info/.dev-stamp

src/easy_robot_control/easy_robot_control.egg-info/.std-stamp: src/easy_robot_control/requirements.txt
	@echo "Installing Python dependencies with pip"
	CXXFLAGS="-fno-fat-lto-objects --param ggc-min-expand=10 --param ggc-min-heapsize=2048" pip install -r src/easy_robot_control/requirements-std.txt --force-reinstall --upgrade
	@touch src/easy_robot_control/easy_robot_control.egg-info/.std-stamp

.PHONY: python-dev
python-dev: src/easy_robot_control/easy_robot_control.egg-info/.dev-stamp

.PHONY: python-std
python-std: src/easy_robot_control/easy_robot_control.egg-info/.std-stamp

.PHONY: rosdep
rosdep: ./src/easy_robot_control/package.xml
	@echo "Installing ROS2 dependencies..."
	- . /opt/ros/$(ROS2_DISTRO)/setup.sh && \
	sudo rosdep init
	- . /opt/ros/$(ROS2_DISTRO)/setup.sh && \
	rosdep update && \
	rosdep install --from-paths src --ignore-src -r

.PHONY: test
test: $(INSTALL_SETUP)
	@echo "Testing through colcon test"
	. $(INSTALL_SETUP) && \
	colcon test --packages-select motion_stack easy_robot_control ros2_m_hero_pkg rviz_basic --event-handlers console_cohesion+

.PHONY: check-dependencies
check-dependencies: $(INSTALL_SETUP)
	@echo "Checking Python dependencies..."
	@for file in $(shell ls src/*/test/test_imports.py); do \
		. $(INSTALL_SETUP) && \
		pytest -q $$file; \
	done

.PHONY: install
install: rosdep python-std $(INSTALL_SETUP)

.PHONY: install-dev
install-dev: rosdep python-dev $(INSTALL_SETUP) check-dependencies html_doc md_doc
