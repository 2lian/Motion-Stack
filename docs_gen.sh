#!/bin/bash
# Generate Sphinx docs automatically
cd docs
sphinx-apidoc -o source ../src
make clean
make html
echo "Documentation built at docs/build/html"

