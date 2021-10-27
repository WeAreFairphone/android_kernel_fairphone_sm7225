#!/bin/bash

if [ -z "$1" ]; then
    echo "Usage: $0 init|update"
    exit 1
fi

declare -A modules
modules[techpack/audio]="platform/vendor/opensource/audio-kernel"
modules[techpack/camera]="platform/vendor/opensource/camera-kernel"
modules[techpack/data]="platform/vendor/qcom-opensource/data-kernel"
modules[techpack/display]="platform/vendor/opensource/display-drivers"
modules[techpack/video]="platform/vendor/opensource/video-driver"
modules[drivers/staging/wlan-qc/fw-api]="platform/vendor/qcom-opensource/wlan/fw-api"
modules[drivers/staging/wlan-qc/qca-wifi-host-cmn]="platform/vendor/qcom-opensource/wlan/qca-wifi-host-cmn"
modules[drivers/staging/wlan-qc/qcacld-3.0]="platform/vendor/qcom-opensource/wlan/qcacld-3.0"

url=https://gerrit-public.fairphone.software
branch=kernel/11/fp4

if [ "$1" == "init" ]; then
    for path in "${!modules[@]}"; do
        git subtree add --prefix=$path --squash $url/${modules[$path]} $branch
    done
elif [ "$1" == "update" ]; then
    for path in "${!modules[@]}"; do
        GIT_EDITOR=true git subtree pull --prefix=$path --squash $url/${modules[$path]} $branch
    done
else
    echo "Invalid option: $1"
    exit 1
fi
