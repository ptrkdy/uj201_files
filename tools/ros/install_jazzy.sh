#!/usr/bin/env bash
# Install ROS 2 Jazzy and build the UJ201 description, on Ubuntu 24.04 (noble).
#
# Run it inside the 24.04 distro. It needs sudo and will ask for your password.
#
#     bash install_jazzy.sh
#
# Deliberately uses the plain-HTTP apt line rather than HTTPS. packages.ros.org
# resolves to ftp.osuosl.org, which serves a certificate covering only
# osuosl.org and *.osuosl.org -- so HTTPS to that hostname fails name
# validation. APT verifies packages by GPG signature regardless of transport,
# so HTTP here costs nothing in integrity; the keyring below is what provides
# it, and that is fetched over a working HTTPS host.

set -euo pipefail

REPO_URL="https://github.com/ptrkdy/uj201_files.git"
WS="${HOME}/ros2_ws"

say() { printf '\n\033[1;36m==> %s\033[0m\n' "$*"; }
die() { printf '\n\033[1;31m!! %s\033[0m\n' "$*" >&2; exit 1; }

# --- preconditions ----------------------------------------------------------
. /etc/os-release
say "Distribution: ${PRETTY_NAME}"
[ "${VERSION_CODENAME:-}" = "noble" ] || die \
"ROS 2 Jazzy requires Ubuntu 24.04 (noble); this is ${VERSION_CODENAME:-unknown}.
 On 22.04 (jammy) the supported release is Humble -- rerun with ros-humble-desktop
 substituted, or install a 24.04 distro and run this there."

curl -sSf -o /dev/null --max-time 15 http://packages.ros.org/ros2/ubuntu/dists/noble/Release \
  || die "Cannot reach the ROS apt repository over HTTP. Check the network first."
say "ROS apt repository reachable"

# --- apt sources ------------------------------------------------------------
say "Enabling universe and installing prerequisites"
sudo apt-get update -qq
sudo apt-get install -y -qq software-properties-common curl gnupg ca-certificates
sudo add-apt-repository -y universe

say "Adding the ROS 2 apt repository"
sudo install -d -m 0755 /usr/share/keyrings
# raw.githubusercontent.com has a valid certificate, so the key comes over HTTPS.
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
     -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null

# --- ROS --------------------------------------------------------------------
say "Installing ros-jazzy-desktop (this is the long part -- a couple of GB)"
sudo apt-get update -qq
sudo apt-get install -y ros-jazzy-desktop \
     ros-jazzy-xacro ros-jazzy-joint-state-publisher-gui \
     ros-jazzy-urdf-tutorial python3-colcon-common-extensions liburdfdom-tools

grep -qF 'source /opt/ros/jazzy/setup.bash' "${HOME}/.bashrc" \
  || echo 'source /opt/ros/jazzy/setup.bash' >> "${HOME}/.bashrc"

# ROS's setup files reference unset variables (AMENT_TRACE_SETUP_FILES and
# friends), so -u has to be off while sourcing them.
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
set -u
say "ROS 2 ${ROS_DISTRO} installed"

# --- workspace --------------------------------------------------------------
# Kept on the Linux filesystem: a colcon build over /mnt/c is painfully slow.
say "Setting up ${WS}"
mkdir -p "${WS}/src"
if [ -d "${WS}/uj201_files/.git" ]; then
  git -C "${WS}/uj201_files" pull --ff-only
else
  git clone "${REPO_URL}" "${WS}/uj201_files"
fi

say "Generating the description package"
cd "${WS}/uj201_files"
python3 tools/host/urdf_build.py arm_assembly_model.json -o "${WS}/src" \
        --overrides overrides.json --mesh-frame local || true
python3 tools/host/urdf_lint.py "${WS}/src/arm_assembly_description/urdf/arm_assembly.xacro"

# check_urdf on the flat description needs no package index, so it can run now.
say "Validating the flat description"
check_urdf "${WS}/src/arm_assembly_description/urdf/arm_assembly.urdf"

say "Building the workspace"
cd "${WS}"
colcon build --symlink-install

# xacro resolves $(find ...) through the ament index, which only knows this
# package once the workspace is built and sourced -- hence the ordering.
say "Validating the xacro through the real toolchain"
set +u
# shellcheck disable=SC1091
source "${WS}/install/setup.bash"
set -u
xacro "${WS}/src/arm_assembly_description/urdf/arm_assembly.xacro" > /tmp/arm_assembly.urdf
check_urdf /tmp/arm_assembly.urdf

cat <<EOF

$(printf '\033[1;32mDone.\033[0m')

    source ${WS}/install/setup.bash
    ros2 launch arm_assembly_description display.launch.py

WSLg is present, so RViz opens natively -- no X server needed.
EOF
