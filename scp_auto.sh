#! /bin/sh
project_path=$(
    cd $(dirname $0)
    pwd
)
#配置目标设备的地址和文件还有密码
expect -c "
    spawn scp -r Build/bin/helloworld root@192.168.1.177:/root
    expect {
        \"yes/no\" {send \"yes\r\";}
        \"*assword\" {set timeout 300; send \"1\r\"; exp_continue;}
       
    }
expect eof"
