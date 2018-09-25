#!/usr/bin/env bash
cd web
python -m SimpleHTTPServer ${1-8000}