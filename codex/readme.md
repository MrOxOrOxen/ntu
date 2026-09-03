codex文件夹用于记录在nscc aspire 2a环境下，在vscode remote ssh中部署codex的过程。

Last updated on Sep 4, 2026.

Copyright © 2026 MARS Lab, Nanyang Technological University, Singapore.

All rights reserved.

参考：

- 实验室指南：在 NSCC ASPIRE 2A 上通过 VS Code Remote-SSH 使用 Codex.html（*Author: Nianbing Su $ Yizhou Liu @ MARS Lab*）
- [NSCC ASPIRE 2A FAQ](https://help.nscc.sg/aspire2a/faqs/)
- [OpenAI Codex CLI](https://learn.chatgpt.com/docs/codex/cli)
- [VS Code Remote - SSH](https://code.visualstudio.com/docs/remote/ssh)


# 注意事项

1. 仅在macOS系统中进行过测试；
2. 仅考虑过直连校园网的情况，未考虑通过ssh转发以及通过vpn连接校园网的情况（这种情况需要申请vpn访问nscc的权限，可以查看nscc文件夹）;
3. 其余平台（如Windows）以及任何报错请查看前面写到的html文件。

# Step 0: 后文出现的常量值

- USER: nscc用户名
- 本地proxy端口，建议用8899，后文统一用8899
- nscc端临时端口，建议用17891，后文统一用17891

如果以上端口不可用，也可以自己选择新的端口，只要端口可用即可。

# Step 1: 启用本地proxy

1. 如果是第一次启动：

```
python3 -m venv "$HOME/.local/share/nscc-codex-proxy"
"$HOME/.local/share/nscc-codex-proxy/bin/pip" install --upgrade pip proxy.py
```

2. 启动本地proxy：

```
"$HOME/.local/share/nscc-codex-proxy/bin/proxy" \
  --hostname 127.0.0.1 \
  --port 8899 \
  --num-workers 1 \
  --num-acceptors 1
```

3. 不要关闭这个终端，在另一个终端验证本地代理是否成功：

```
curl -sS -o /dev/null -w 'HTTP_%{http_code}\n' \
  --connect-timeout 10 \
  --max-time 20 \
  --proxy http://127.0.0.1:8899 \
  https://api.openai.com/v1/models
```

如果输出http401则为网络路径已经连通。

4. 本地写入ssh配置

```
touch ~/.ssh/config
chmod 600 ~/.ssh/config
mkdir -p ~/.ssh/sockets
chmod 700 ~/.ssh/sockets
```

编辑ssh config文件：

nano ~/.ssh/config, 写入：

```
Host 103.72.192.6
  HostName 103.72.192.6
  User USER
  PreferredAuthentications keyboard-interactive,password
  PubkeyAuthentication no
  ServerAliveInterval 60
  ServerAliveCountMax 3
  ExitOnForwardFailure yes
  RemoteForward 127.0.0.1:17891 127.0.0.1:8899
  ControlMaster auto
  ControlPath ~/.ssh/sockets/%C
  ControlPersist 600
```

5. 本地测试

先停止与所有nscc的ssh连接，然后在本地启动新的master：

```
ssh -MNf 103.72.192.6
ssh -O check 103.72.192.6
ssh nscc "hostname; curl -sS -o /dev/null -w 'HTTP_%{http_code}\n' \
  --connect-timeout 10 --max-time 20 \
  --proxy http://127.0.0.1:17891 \
  https://api.openai.com/v1/models; echo"
```

如果看到login node名称与HTTP 401，则为测试成功。

# Step 2: 在vscode中连接nscc

在vscode中通过ssh连接nscc，确认自己在登录节点而不是计算节点中。

1. 编辑bashrc（nano ~/.bashrc）

```
if [[ "$(hostname -s)" == asp2a-login-* ]]; then
  export HTTP_PROXY="http://127.0.0.1:17891"
  export HTTPS_PROXY="$HTTP_PROXY"
  export http_proxy="$HTTP_PROXY"
  export https_proxy="$HTTPS_PROXY"
  export NO_PROXY="localhost,127.0.0.1,::1"
  export no_proxy="$NO_PROXY"
  export PATH="$HOME/.local/bin:$PATH"
fi
```

完成后source ～/.bashrc使修改生效。

2. 测试bashrc

```
printf 'HTTP_PROXY=%sn' "$HTTP_PROXY"
```

应该显示http://127.0.0.1:17891.

3. 设置vscode server

nano ~/.vscode-server/server-env-setup，若文件不存在则创建。编辑文件：

```
case "$(hostname -s)" in
  asp2a-login-*)
  export HTTP_PROXY="http://127.0.0.1:17891"
  export HTTPS_PROXY="$HTTP_PROXY"
  export http_proxy="$HTTP_PROXY"
  export https_proxy="$HTTPS_PROXY"
  export NO_PROXY="localhost,127.0.0.1,::1"
  export no_proxy="$NO_PROXY"
  export PATH="$HOME/.local/bin:$PATH"
  ;;
esac
```

4. 检查vscode server

```
chmod 600 "$HOME/.vscode-server/server-env-setup"
sh -n "$HOME/.vscode-server/server-env-setup"
```

若无输出，则脚本格式正常。

# Step 3: 服务器端安装codex

1. 安装codex

确保自己在登录节点，运行：

```
curl -fsSL https://chatgpt.com/codex/install.sh | sh
export PATH="$HOME/.local/bin:$PATH"
command -v codex
codex --version
codex login status
```

如果未登录或者cookie失效，运行codex login.

2. 测试codex

```
codex exec --ephemeral --sandbox read-only --skip-git-repo-check -C /tmp \
'Reply with exactly: READY. Do not use tools, run commands, or modify files.'
```

如果看到回复为READY，则codex正常工作。

# Step 4: 后续步骤

重启vscode之后即可在侧边栏使用codex扩展。

后续每次ssh服务器之前，需要进行以下工作：

1. 启动本地proxy

```
"$HOME/.local/share/nscc-codex-proxy/bin/proxy" \
  --hostname 127.0.0.1 \
  --port 8899 \
  --num-workers 1 \
  --num-acceptors 1
```

不要关闭这个终端。

2. 运行以下命令

```
ssh -MNf 103.72.192.6
ssh -O check 103.72.192.6
ssh nscc "hostname; curl -sS -o /dev/null -w 'HTTP_%{http_code}\n' \
  --connect-timeout 10 --max-time 20 \
  --proxy http://127.0.0.1:17891 \
  https://api.openai.com/v1/models; echo"
```

出现hostname和HTTP 401之后即可使用vscode的ssh.