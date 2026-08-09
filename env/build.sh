#!/usr/bin/env bash

set -Eeuo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace_dir="$(cd "${script_dir}/.." && pwd)"
venv_dir="${GRASP_VENV_DIR:-${workspace_dir}/.venv}"
profile_id="$(printf '%s' "${workspace_dir}" | sha256sum | cut -c1-12)"
output_dir="${GRASP_BUILD_ROOT:-${workspace_dir}/.portable/${profile_id}}"
wheel_dir="${script_dir}/wheels"
lock_file="${script_dir}/python-requirements.lock"
agx_workspace_dir="${AGX_ARM_WS:-$(cd "${workspace_dir}/.." && pwd)/agx_arm_ws}"

"${script_dir}/check-system.sh"

echo "校验离线依赖"
(
    cd "${script_dir}"
    sha256sum --check SHA256SUMS
)

echo "加载 AGX underlay: ${agx_workspace_dir}"
set +u
# shellcheck disable=SC1090
source "${agx_workspace_dir}/env.sh"
set -u

if [[ ! -x "${venv_dir}/bin/python" ]]; then
    echo "创建虚拟环境: ${venv_dir}"
    /usr/bin/python3 -m venv --system-site-packages "${venv_dir}"
fi

if ! grep -q '^include-system-site-packages = true$' "${venv_dir}/pyvenv.cfg"; then
    echo "错误: ${venv_dir} 未启用 system-site-packages，无法访问 ROS 2 Python 包。" >&2
    echo "请移走该目录后重新运行此脚本。" >&2
    exit 1
fi

export PYTHONNOUSERSITE=1

echo "从 env/wheels 离线安装 Python 依赖"
"${venv_dir}/bin/python" -m pip install \
    --disable-pip-version-check \
    --no-index \
    --find-links "${wheel_dir}" \
    --requirement "${lock_file}"

# shellcheck disable=SC1091
set +u
source "${venv_dir}/bin/activate"
set -u
export LC_NUMERIC=en_US.UTF-8

mkdir -p "${output_dir}/build" "${output_dir}/install" "${output_dir}/log"

echo "编译工作区: ${workspace_dir}"
echo "编译输出: ${output_dir}"
cd "${workspace_dir}"
colcon --log-base "${output_dir}/log" build \
    --symlink-install \
    --build-base "${output_dir}/build" \
    --install-base "${output_dir}/install"

echo
echo "编译完成。加载环境: source \"${workspace_dir}/env.sh\""
