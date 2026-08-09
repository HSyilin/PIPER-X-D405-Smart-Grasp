#!/usr/bin/env bash

_grasp_env_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${_grasp_env_dir}/env/activate.sh"
unset _grasp_env_dir
