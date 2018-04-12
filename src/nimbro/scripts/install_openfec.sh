#!/bin/bash
# Download and install OpenFEC (use with a little bit of caution as it deletes any existing installation in the same spot)

set -e # Exit on any failure

openfecver=openfec_v1.4.2
openfecfile="${openfecver//./_}".tgz

tmpdir="/tmp/openfec"
[[ -d "$tmpdir" ]] && rm -rf "$tmpdir"
mkdir -p "$tmpdir"
cd "$tmpdir"

set -o xtrace # Show commands

wget -O "$openfecfile" http://openfec.org/files/"$openfecfile" && tar -xz -f "$openfecfile"
cd "$openfecver"

mkdir -p build_debug build_release
( cd build_debug && cmake .. -DDEBUG:STRING=ON && make )
( cd build_release && cmake .. -DDEBUG:STRING=OFF && make )

openfec="/opt/$openfecver"
[[ -e "$openfec" ]] && sudo rm -rf "$openfec"
sudo mv "$tmpdir/$openfecver" /opt/

[[ -d "$tmpdir" ]] && rm -rf "$tmpdir"

set +o xtrace # Do not show commands
set +e # Do not exit on any failure

echo
echo "You can now find OpenFEC at $openfec"
echo
# EOF