nscc文件夹用于记录在校园网以及vpn情况下连接nscc的方法，以及nscc内的指令与文件结构。

Last updated on Sep 4, 2026.

Copyright © 2026 MARS Lab, Nanyang Technological University, Singapore.

All rights reserved.

参考：

- nscc.docx（Author: Jiahua Dong @ MARS Lab）
- [Using NTU JumpHost to NSCC ASPIRE-2A](https://entuedu.sharepoint.com/teams/ntuhpcusersgroup2/SitePages/Using-NTU-JumpHost-to-NSCC-ASPIRE-2A.aspx)

以下步骤能够完整实现的前提是：已经有了NTU的vpn权限。

# Step 0: 后文出现的常量值

- USER: nscc账号用户名
- USER_NTU: ntu账号用户名

# Step 1: 注册nscc账号

访问https://user.nscc.sg/saml/index.php，以NTU身份登录，设置ssh的password.

说明：

1. ssh的key暂时无法使用。
2. nscc强制要求必须使用密码来ssh，无法通过配置公钥与私钥来免密ssh连接。
3. 经过实测，配置ssh短期免密会出问题。
4. 每90天nscc会重置一次密码，需要再次去网站上手动修改。

# Step 2: vscode连接

ssh的ip：103.72.192.6

1. 配置ssh config

在本地nano ~/.ssh/config:

```
Host 103.72.192.6
    HostName 103.72.192.6
    User USER
    PreferredAuthentications keyboard-interactive,password
    PubkeyAuthentication no
    ServerAliveInterval 60
    ServerAliveCountMax 3
```

nano之后 chmod 600 ~/.ssh/config即可。

2. scratch文件夹

ssh进入之后会发现一个scratch文件夹，这其实是一个软链接文件夹，你所看到的绝对路径为：

```
/home/users/ntu/USER/scratch/
```

但实际上是：

```
/scratch/home/users/ntu/USER/
```

写绝对路径的时候写哪一种都行。

需要注意的是，scratch文件夹设置了30天未读文件就自动清理的机制。但同时，scratch文件夹的存储空间大小远超剩下的所有文件夹（剩下的所有文件夹加一起只有50GB），所以需要把模型、checkpoint以及log等放在scratch文件夹里，但需要注意如果有需要的话要手动备份一下。

3. 计算节点申请

临时节点申请：这种申请方式仅适用于当前终端，当该终端退出时计算节点自动释放。

终端运行：

```
qsub -I -P personal -q normal -l select=1:ncpus=8:ngpus=1:mem=80gb -l walltime=06:00:00
```

其中：

- -P personal: 表示使用自己的额度
- -q normal: 标准计算模式
- -l中：
    - select: 分配几个计算节点
    - ncpus: cpu core数量
    - ngpus: gpu core数量
    - mem: 分配ram大小（但好像无论填多少都是写着分配110gb实际是40gb）
    - walltime: 超时时间

分配之后会得到一个计算节点的hostname. 如果想退出计算节点，输入exit或者按ctrl+D即可。

长期节点申请：

如果想要退出终端还能让训练任务正常进行，可以使用nohup，也可以直接写一个pbs脚本（即扩展名为pbs）：

```
#!/bin/bash

#PBS -N abot_train
#PBS -P personal
#PBS -q normal
#PBS -l select=1:ncpus=8:ngpus=1:mem=80gb
#PBS -l walltime=06:00:00
#PBS -j oe
#PBS -o path_to_log

...
```

其中，-N为任务名称，-j oe表示保存普通日志与错误日志，-o表示日志文件路径，...表示后续想要一起执行的操作。pbs脚本提交之后，会有一个pbs的id.

4. pbs相关指令：

- 查询当前用户pbs任务状态：qstat -u USER，其中Q表示排队，R表示运行，E表示报错；
- 提交pbs任务：qsub xxx.pbs；
- 停止pbs任务：qdel id，其中id为提交pbs任务时显示的id，也可以通过查询pbs任务状态来查询。

5. pbs日志：

pbs脚本的日志会在pbs脚本运行结束之后才生成，如果想实时查看日志，需要在pbs脚本里：

```
bash path_to_log > path_to_realtime_log 2>&1
```

然后通过tail -f path_to_realtime_log即可实时查看日志。

# Step 3: vpn连接nscc

获得vpn权限之后，需要发送邮件至hpcsupport@ntu.edu.sg来获取vpn连接nscc的权限。

在ssh config中写入：

```
Host 172.21.26.100
  HostName 172.21.26.100
  User USER_NTU
  Ciphers aes128-ctr
  MACs hmac-sha2-256
```

即可通过ssh跳板连接连接到nscc.
