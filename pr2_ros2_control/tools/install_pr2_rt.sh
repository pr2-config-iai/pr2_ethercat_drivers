#!/bin/sh
# Install pr2-rt-exec (setuid root) and pr2-rt into /usr/local/bin.
# Run once per machine or container (again if the container is recreated):
#
#   ./install_pr2_rt.sh [group]
#
# Only members of <group> (default: your primary group) can run pr2-rt-exec. See decision D11 in
# pr2_ros2/docs/decisions.md and the comment at the top of pr2-rt-exec.c.
set -eu
here=$(cd "$(dirname "$0")" && pwd)
group=${1:-$(id -gn)}
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT
cc -O2 -Wall -Wextra -Werror -o "$tmp/pr2-rt-exec" "$here/pr2-rt-exec.c" -lcap
sudo install -o root -g "$group" -m 4750 "$tmp/pr2-rt-exec" /usr/local/bin/pr2-rt-exec
sudo install -o root -g root -m 0755 "$here/pr2-rt" /usr/local/bin/pr2-rt
ls -l /usr/local/bin/pr2-rt-exec /usr/local/bin/pr2-rt
