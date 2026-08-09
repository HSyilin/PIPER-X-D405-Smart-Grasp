#!/usr/bin/env bash

_grasp_script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_grasp_workspace_dir="$(cd "${_grasp_script_dir}/.." && pwd)"
_grasp_venv_dir="${GRASP_VENV_DIR:-${_grasp_workspace_dir}/.venv}"
_grasp_profile_id="$(printf '%s' "${_grasp_workspace_dir}" | sha256sum | cut -c1-12)"
_grasp_output_dir="${GRASP_BUILD_ROOT:-${_grasp_workspace_dir}/.portable/${_grasp_profile_id}}"
_grasp_workspace_install_dir="${_grasp_workspace_dir}/install"
_grasp_portable_install_dir="${_grasp_output_dir}/install"
_grasp_agx_workspace_dir="${AGX_ARM_WS:-$(cd "${_grasp_workspace_dir}/.." && pwd)/agx_arm_ws}"
_grasp_agx_venv_dir="${AGX_ARM_VENV_DIR:-${_grasp_workspace_dir}/.agx_arm_venv}"
_grasp_pyagxarm_dir="${AGX_PYAGXARM_DIR:-$(cd "${_grasp_workspace_dir}/.." && pwd)/pyAgxArm}"

if [[ ! -f "${_grasp_agx_workspace_dir}/env.sh" ]]; then
    echo "错误: 找不到 AGX 工作区: ${_grasp_agx_workspace_dir}" >&2
    return 1
fi
if [[ -f "${_grasp_agx_venv_dir}/bin/activate" ]]; then
    export AGX_ARM_VENV_DIR="${_grasp_agx_venv_dir}"
fi
if [[ ! -f "${_grasp_venv_dir}/bin/activate" ]]; then
    echo "错误: 找不到虚拟环境，请先运行 ${_grasp_script_dir}/build.sh" >&2
    return 1
fi

if [[ -f "${_grasp_workspace_install_dir}/smart_grasp_bringup/share/smart_grasp_bringup/package.sh" ]]; then
    _grasp_install_dir="${_grasp_workspace_install_dir}"
elif [[ -f "${_grasp_portable_install_dir}/smart_grasp_bringup/share/smart_grasp_bringup/package.sh" ]]; then
    _grasp_install_dir="${_grasp_portable_install_dir}"
else
    echo "错误: 找不到可用的 smart_grasp_bringup 编译结果，请先运行 ${_grasp_script_dir}/build.sh" >&2
    return 1
fi

case $- in
    *u*) _grasp_restore_nounset=1; set +u ;;
    *) _grasp_restore_nounset=0 ;;
esac

if [[ -f "${_grasp_agx_workspace_dir}/install/setup.bash" ]]; then
    # Prefer the already-built install tree. The upstream env.sh insists on a
    # portable build root, which breaks on prebuilt workspaces.
    # shellcheck disable=SC1090
    source "${_grasp_agx_workspace_dir}/install/setup.bash"
else
    # Fallback to the upstream entry point when a portable build root exists.
    # shellcheck disable=SC1090
    source "${_grasp_agx_workspace_dir}/env.sh"
fi
# shellcheck disable=SC1090
source "${_grasp_venv_dir}/bin/activate"
# shellcheck disable=SC1090
source "${_grasp_install_dir}/setup.bash"

if [[ "${_grasp_restore_nounset}" == 1 ]]; then
    set -u
fi

export LC_NUMERIC=en_US.UTF-8
export PYTHONNOUSERSITE=1
export GRASP_WS="${_grasp_workspace_dir}"
_grasp_agx_pythonpath="${_grasp_install_dir}/agx_arm_msgs/local/lib/python3.10/dist-packages:${_grasp_agx_workspace_dir}/install/agx_arm_msgs/local/lib/python3.10/dist-packages:${_grasp_agx_workspace_dir}/install/agx_arm_ctrl/lib/python3.10/site-packages:${_grasp_pyagxarm_dir}:${HOME}/.local/lib/python3.10/site-packages"
if [[ -n "${PYTHONPATH:-}" ]]; then
    export PYTHONPATH="${_grasp_agx_pythonpath}:${PYTHONPATH}"
else
    export PYTHONPATH="${_grasp_agx_pythonpath}"
fi

unset _grasp_script_dir _grasp_workspace_dir _grasp_venv_dir
unset _grasp_profile_id _grasp_output_dir _grasp_workspace_install_dir _grasp_portable_install_dir
unset _grasp_install_dir _grasp_agx_workspace_dir _grasp_agx_venv_dir
unset _grasp_restore_nounset _grasp_agx_pythonpath
unset _grasp_pyagxarm_dir
