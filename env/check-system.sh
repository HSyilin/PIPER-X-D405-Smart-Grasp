#!/usr/bin/env bash

set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace_dir="$(cd "${script_dir}/.." && pwd)"
package_file="${script_dir}/system-apt-packages.txt"
agx_workspace_dir="${AGX_ARM_WS:-$(cd "${workspace_dir}/.." && pwd)/agx_arm_ws}"
missing=()

if [[ "$(uname -m)" != "x86_64" ]]; then
    echo "错误: 离线 wheel 适用于 x86_64，当前架构是 $(uname -m)。" >&2
    exit 1
fi

if [[ ! -r /etc/os-release ]]; then
    echo "错误: 无法识别操作系统。" >&2
    exit 1
fi

# shellcheck disable=SC1091
source /etc/os-release
if [[ "${ID:-}" != "ubuntu" || "${VERSION_ID:-}" != "22.04" ]]; then
    echo "错误: 需要 Ubuntu 22.04，当前是 ${PRETTY_NAME:-unknown}。" >&2
    exit 1
fi

while IFS= read -r package; do
    [[ -z "${package}" || "${package}" == \#* ]] && continue
    if ! dpkg-query -W -f='${Status}' "${package}" 2>/dev/null | grep -q 'ok installed'; then
        missing+=("${package}")
    fi
done < "${package_file}"

if ((${#missing[@]})); then
    echo "缺少以下系统/ROS 依赖:" >&2
    printf '  %s\n' "${missing[@]}" >&2
    echo >&2
    printf '联网后安装: sudo apt install' >&2
    printf ' %q' "${missing[@]}" >&2
    echo >&2
    exit 1
fi

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    echo "错误: 找不到 /opt/ros/humble/setup.bash。" >&2
    exit 1
fi

if [[ "$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')" != "3.10" ]]; then
    echo "错误: 离线环境需要 Python 3.10。" >&2
    exit 1
fi

if [[ ! -f "${agx_workspace_dir}/env.sh" ]]; then
    echo "错误: 找不到 AGX 工作区: ${agx_workspace_dir}" >&2
    echo "可通过 AGX_ARM_WS 指定它的位置。" >&2
    exit 1
fi

echo "系统检查通过: Ubuntu 22.04 / ROS 2 Humble / Python 3.10 / x86_64"
echo "AGX underlay: ${agx_workspace_dir}"
