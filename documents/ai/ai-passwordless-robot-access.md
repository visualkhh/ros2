# AI(Claude Code)가 로봇 Pi를 비밀번호 없이 다루게 하기

AI 에이전트는 비밀번호 프롬프트에 답할 수 없습니다 (`ssh` 비번, `sudo` 비번 모두).
그래서 **사람이 처음 한 번만** 비밀번호를 넣어 아래 4가지를 깔아 두면, 그 뒤로는 AI가
`ssh pak '...'`로 설치·빌드·업로드·시험을 직접 합니다.

| 단계 | 사람이 1회 | 그 뒤 AI가 하는 것 |
|---|---|---|
| 1. 이름으로 찾기 (mDNS) | Pi에 `avahi-daemon` | `ssh pak@pak.local` (IP 몰라도 됨) |
| 2. SSH 키 | `ssh-copy-id` (Pi 비번 1회) | 비번 없이 접속 |
| 3. SSH 별칭 | `~/.ssh/config`에 `Host pak` | `ssh pak '명령'` |
| 4. sudo 비번 없이 | `/etc/sudoers.d/90-pak` (sudo 비번 1회) | `sudo apt install` · udev · 재부팅 |

기준: PC `pank@pank-Legion-Pro-5-16IAX10` → Pi `pak@pak.local` (Raspberry Pi 4, Ubuntu 26.04). 2026-10-27 실측.

## 1. mDNS (`pak.local`)

첫 부팅에 인터넷이 없으면 cloud-init의 `avahi-daemon` 설치가 실패해서 `pak.local`이 안 풀립니다.
IP로 한 번 들어가서 설치합니다 (IP 찾기: [SSH 문서 7장](../raspberry/ssh-pak-local-troubleshooting.md)).

```bash
ssh pak@<IP>
sudo apt update && sudo apt install -y avahi-daemon libnss-mdns
sudo systemctl enable --now avahi-daemon
```

PC 확인: `getent hosts pak.local` → `10.2.12.198 pak.local`

## 2. SSH 키 (비번 없이 접속)

```bash
ls ~/.ssh/id_ed25519.pub || ssh-keygen -t ed25519
ssh-copy-id -i ~/.ssh/id_ed25519.pub pak@pak.local   # ← Pi 비번 1회 (사용자명 pak@ 꼭 붙일 것)
```

`pak.local`로 처음 접속하면 호스트 키를 묻습니다. AI는 `yes`를 못 치므로(`BatchMode`에서는
`Host key verification failed`로 끝남) 사람이 한 번 접속해 두거나, 같은 키인지 확인하고 등록합니다:

```bash
# IP로 이미 등록된 키와 pak.local의 키가 같을 때만 등록
a=$(ssh-keyscan -t ed25519 pak.local 2>/dev/null | awk '{print $3}')
b=$(ssh-keygen -F 10.2.12.198 | awk '/ed25519/{print $3}')
[ -n "$a" ] && [ "$a" = "$b" ] && ssh -o StrictHostKeyChecking=accept-new pak@pak.local true
```

## 3. SSH 별칭 (`~/.ssh/config`)

```
Host pak
    HostName pak.local
    User pak
    IdentityFile ~/.ssh/id_ed25519
    IdentitiesOnly yes
```

`chmod 600 ~/.ssh/config`. 이제 `ssh pak`. mDNS가 안 되는 망이면 `HostName`을 IP로.

## 4. sudo 비번 없이

```bash
ssh -t pak 'echo "pak ALL=(ALL) NOPASSWD:ALL" | sudo tee /etc/sudoers.d/90-pak && sudo chmod 440 /etc/sudoers.d/90-pak && sudo visudo -c'
# ← sudo 비번 1회. 마지막에 parsed OK
```

`-t`가 있어야 비번 프롬프트가 뜹니다. Raspberry Pi Imager의 기본 사용자도 원래 이렇게(NOPASSWD) 만들어지는데,
이 SD카드는 user-data에 `sudo: null`이 들어가 비번을 묻게 되어 있었습니다.

## 5. AI가 되는지 확인하는 법

AI는 프롬프트가 뜨면 멈추므로 **`BatchMode=yes`와 `sudo -n`** 으로 "물어보면 바로 실패"하게 해서 확인합니다.

```bash
ssh -o BatchMode=yes pak hostname          # pak → 2·3번 OK
ssh -o BatchMode=yes pak 'sudo -n true && echo SUDO_OK'   # SUDO_OK → 4번 OK
```

| 실패 메시지 | 원인 | 고칠 곳 |
|---|---|---|
| `Could not resolve hostname pak.local` | avahi 없음 / mDNS 막는 망 | 1번 또는 `HostName`을 IP로 |
| `Host key verification failed` | 이 이름으로 처음 접속 | 2번 끝 (사람이 한 번 접속) |
| `Permission denied (publickey,password)` | 키 미등록 · 사용자명 틀림(`pank@`로 붙음) | 2번 (`pak@` 붙이기) |
| `sudo: interactive authentication is required` | sudoers 미설정 | 4번 |

## 6. 보안 — 알고 쓰기

- 4번 이후에는 **PC의 `~/.ssh/id_ed25519`를 가진 사람(과 그 PC에서 도는 AI)이 Pi의 root**입니다. 키 파일을 지키세요.
- 키 접속이 되는 걸 확인했으면 Pi의 비밀번호 로그인을 끄면 더 안전합니다 (네트워크의 다른 사람이 비번을 시도할 수 없음):
  ```bash
  ssh pak 'echo "PasswordAuthentication no" | sudo tee /etc/ssh/sshd_config.d/99-nopass.conf && sudo systemctl restart ssh'
  ```
  끄기 전에 반드시 `ssh -o BatchMode=yes pak hostname`이 되는지 먼저 확인 (안 되면 모니터 없이 못 들어감).
- AI에게 맡길 때도 **되돌리기 어려운 것**(펌웨어 업로드로 기존 펌웨어 지우기, 디스크 포맷, 비번 로그인 끄기)은
  실행 전에 사람에게 묻도록 합니다.

되돌리기:

```bash
ssh -t pak 'sudo rm /etc/sudoers.d/90-pak'                 # sudo 다시 비번 묻게
key=$(cut -d" " -f2 ~/.ssh/id_ed25519.pub)                # 이 PC 키 제거 (마지막에 — 이후 비번 필요)
ssh pak "grep -vF '$key' ~/.ssh/authorized_keys > ~/.ssh/ak.new && mv ~/.ssh/ak.new ~/.ssh/authorized_keys"
```

## 7. 이걸로 AI가 하는 일 (이 프로젝트)

- 설치: [Lyrical (Pi)](../ros/lyrical/lyrical-setup-ubuntu26.04.md) — apt · swap · `.bashrc` · `colcon build`
- 펌웨어: [OpenCR](../opencr/opencr-setup-ubuntu26.04.md) — `setup.sh` · [udev](../raspberry/udev-setup.md) · `compile.sh` · `upload.sh` · `test-firmware`
- 코드 갱신: PC에서 push → `ssh pak 'cd ~/ros2 && git pull'` (Pi는 GitHub 키 없이 https로 pull만)
- 운영: PC에서 그대로 `tool/run-cli` · `tool/run-rviz` — 켜기/끄기·지도처럼 Pi 프로세스·파일이 필요한 것만 tool이 이 SSH 키로 Pi에 넘김 (`pak@pak.local`)
