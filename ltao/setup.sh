# run in a terminal with: . setup.sh
export MOUNT_MODEL=MOUNT_FDR_8kHz
export FEM_REPO=/home/ubuntu/mnt/20240401_1605_zen_30_M1_202110_ASM_202403_Mount_202305_IDOM_concreteAndFoundation_M1Fans//
export GMT_MODES_PATH=$FEM_REPO #/home/ubuntu/CEO/gmtMirrors/
export SCOPE_SERVER_IP=`ec2metadata | sed -n 's/^local-ipv4: \(.*\)/\1/p'`
export FLOWCHART=dot
export TO_DOT=1
