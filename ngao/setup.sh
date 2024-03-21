# run in a terminal with: . setup.sh
export GMT_MODES_PATH=/home/ubuntu/mnt/ceo
export FLOWCHART=sfdp
export SCOPE_SERVER_IP=`ec2metadata | sed -n 's/^local-ipv4: \(.*\)/\1/p'`