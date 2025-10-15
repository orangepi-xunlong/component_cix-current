#!/bin/bash

#  Copyright 2024 Cix Technology Group Co., Ltd.
#  All Rights Reserved.
#
#  The following programs are the sole property of Cix Technology Group Co., Ltd.,
#  and contain its proprietary and confidential information.
#

WORKSPACE="$(realpath --no-symlinks "$(dirname "${BASH_SOURCE[0]}")")"

sed -i 's|http://mirrors.ustc.edu.cn/debian|https://repo.huaweicloud.com/debian|g' /etc/apt/sources.list
sed -i 's|https://mirrors.ustc.edu.cn/debian|https://repo.huaweicloud.com/debian|g' /etc/apt/sources.list
apt-get update

# install cix deb packages

# debs=`find "/debs" -maxdepth 1 -name "*.deb" | xargs echo`
# arr=($debs)
# len=${#arr[@]}
# for ((i=0;i<$len;i++))
# do
#    apt install ${arr[$i]}
# done

if [[ -L /usr/lib/policykit-1/polkit-agent-helper-1 ]]; then
    chmod 5755 /usr/lib/policykit-1/polkit-agent-helper-1
fi
if [ 1 == 6 ]; then
    if [[ -e $(ls ${WORKSPACE}/cix-gstreamer*.deb 2>/dev/null) ]]; then
        rm -rf ${WORKSPACE}/cix-gstreamer*.deb 
    fi
    if [[ -e $(ls ${WORKSPACE}/cix-firmware*.deb 2>/dev/null) ]]; then
        dpkg -i --force-overwrite ${WORKSPACE}/cix-firmware*.deb
    fi
fi
export DEBIAN_FRONTEND=noninteractive
apt -y --allow-downgrades install ${WORKSPACE}/*.deb
systemctl mask swap.target
systemctl enable cix_resume.service
systemctl enable cix_resume_prepare.service
rm -f /cix-gnome-core-1.43_arm64.deb


