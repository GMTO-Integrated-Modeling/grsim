# run in a terminal with: . setup.sh
mount-s3 --cache ~/.s3-cache/ gmto.im.grim ~/mnt
export GMT_MODES_PATH=/home/ubuntu/mnt/ceo
export FEM_REPO=/home/ubuntu/mnt/20230131_1605_zen_30_M1_202110_ASM_202208_Mount_202111/
export MOUNT_MODEL=MOUNT_PDR_8kHz
export FLOWCHART=sfdp
export SCOPE_SERVER_IP=`ec2metadata | sed -n 's/^local-ipv4: \(.*\)/\1/p'`