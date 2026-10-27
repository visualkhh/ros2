# `ssh pak@pak.local` 실패 — SD카드 PC에 꼽아 확인하기

증상: `ssh: Could not resolve hostname pak.local: Name or service not known`
같은 SSID, 같은 hostname(`pak`)으로 설정했는데도 접속 안 됨.

관련: [SD카드 OS 설치](raspberry-pi-ubuntu-sdcard-os-install.md) · 이미지: `image/img.png`, `image/img_1.png`, `image/img_2.png`

## 1. 결론부터 (3가지 중 하나)

1. Pi가 Wi-Fi에 못 붙음 (SSID 오타 / 국가코드 GB / 5GHz / 사내망 격리)
2. 붙었는데 `.local`이 안 풀림 (Avahi 문제 / AP Isolation / hostname 중복)
3. SSH 자체가 꺼짐 (Imager 서비스탭 미체크)

SD카드를 PC에 다시 꼽으면 1번과 3번은 부팅 없이 확인 가능.

## 2. SD카드 PC에 꼽아 체크

Pi 전원 OFF → SD카드 뽑아 Ubuntu PC에 삽입.

```bash
lsblk -p
# 예: sdb  29.7G  <- SD카드
#     sdb1 vfat  system-boot   (설정 파일 여기)
#     sdb2 ext4  writable      (본체 루트)

# 자동 마운트 안 되면 수동으로
udisksctl mount -b /dev/sdb1
ls /media/$USER/system-boot/
# 있어야 할 것: config.txt  cmdline.txt  network-config  user-data  meta-data
```

### 2-1. Wi-Fi 설정 확인 (가장 흔함)

```bash
cat /media/$USER/system-boot/network-config
```

정상 예 (Imager에서 SSID/비번/국가 넣었을 때):

```yaml
version: 2
ethernets:
  eth0:
    dhcp4: true
    optional: true
wifis:
  wlan0:
    dhcp4: true
    optional: true
    access-points:
      "KANT_Teamsparta":
        password: "실제비번"
```

체크리스트:

- [ ] SSID 철자·대소문자 정확? (`KANT_Teamsparta` — 앞뒤 공백 없음)
- [ ] `password:` 있음? 사내망이 ID+비번(WPA2-Enterprise)이면 Imager 방식(psk)으로는 절대 안 붙음. 폰 핫스팟으로 먼저 테스트.
- [ ] 5GHz only SSID? Pi 4B는 5GHz 되지만 초기 설정은 2.4GHz가 안정적.
- [ ] 숨긴 SSID? Imager `숨겨진 SSID` 체크 안 하면 스캔 안 됨.

국가코드는 `user-data` 쪽에 들어감. 문서 이미지(`image/img_1.png`)처럼 `GB`로 되어 있으면 한국 공유기 채널에서 문제 생김:

```bash
grep -i -A2 "country\|regulatory\|wifi" /media/$USER/system-boot/user-data | head -20
# GB로 되어 있으면 KR로 수정 고려 (아래 3장 참조)
```

### 2-2. hostname / 사용자 / SSH 확인

```bash
# hostname (중복 주의: Pi 2대 다 pak이면 충돌 → pak-2.local로 밀림)
grep -i "hostname\|manage_etc_hosts" /media/$USER/system-boot/user-data | head

# 사용자 pak 있음?
grep -A3 "name: pak" /media/$USER/system-boot/user-data | head -10

# SSH 허용?
grep -i "ssh" /media/$USER/system-boot/user-data | head
# ssh_pwauth: true 여야 비번 접속 가능 (image/img_2.png 서비스탭 대응)
```

`pak`이 두 대면 한 대는 hostname을 `pak2` 등으로 바꿔야 함. SD카드 두 장 구웠으면 두 장 다 확인.

### 2-3. 수정 후 안전 제거

```bash
sync
udisksctl unmount -b /dev/sdb1
udisksctl unmount -b /dev/sdb2 2>/dev/null; echo done
```

파일 직접 고쳐도 되지만, SSID/비번이 틀렸으면 **Imager로 다시 굽는 게 확실**. 직접 수정은 `network-config` 들여쓰기(YAML) 깨지면 부팅 안 됨.

## 3. PC 쪽 mDNS 확인 (SD카드 정상인데도 안 될 때)

```bash
sudo apt install -y avahi-daemon libnss-mdns
systemctl status avahi-daemon --no-pager
getent hosts pak.local
avahi-resolve -n pak.local
ping -c3 pak.local
ip route | grep default
ip neigh
# Pi IP 스캔 (공유기 서브넷에 맞게 대역 수정)
sudo nmap -sn 192.168.1.0/24 | grep -B2 -i -E "raspberry|pi"
# 찾은 IP로 직접 접속
ssh pak@192.168.x.x
ssh pak@192.168.x.x -v  # 안 되면 -v 로그 확인
```

- `getent` 안 나오는데 공유기 관리화면에 Pi IP가 보이면 → AP Isolation(기기 간 차단). 사내/게스트 Wi-Fi가 대표적. 폰 핫스팟이나 집 공유기로 PC+Pi 둘 다 옮겨서 테스트.
- IP는 보이는데 `.local`만 안 되면 IP로 접속하면 됨: `ssh pak@192.168.x.x -v`

## 4. Pi 본체에서 확인 (모니터+키보드 있을 때)

```bash
hostnamectl
ip a   # wlan0에 IP 붙었는지
ping -c3 8.8.8.8
ping -c3 pank-Legion-Pro-5-16IAX10.local
systemctl status ssh avahi-daemon --no-pager
sudo systemctl enable --now ssh
```

HDMI는 전원 켜기 전에 먼저 연결 (나중에 꼽으면 화면 안 나옴).

## 5. 다시 굽기 체크리스트

- [ ] Imager 장치: `Raspberry Pi 4`, OS: `Ubuntu Server 22.04.5 LTS (64-bit)` — 26.04 아님 (TurtleBot3는 22.04)
- [ ] 일반탭: hostname 유일하게 (`pak`, `pak2`…), 사용자 `pak`, 시간대 `Asia/Seoul`, 무선 LAN 국가 `KR`
- [ ] 서비스탭: `SSH 사용` + `비밀번호 인증 사용`
- [ ] Wi-Fi: 일반 공유기/핫스팟으로 먼저 성공시킨 뒤 사내망 시도
- [ ] 첫 부팅 2분 대기 (빨간 LED 고정 + 녹색 LED 깜빡임)

## 6. `network-config` 함정 2개 (실측)

- `password:`에 64자리 16진수 해시가 들어가 있으면 의심. 폰에 치는 비번이 사람이 읽을 수 있는 문장이면 해시가 아니라 평문을 넣어야 함. Imager는 평문을 그대로 씀.
- 사내망이 ID+비번(WPA2-Enterprise)이면 `password:` 방식(psk)으로는 절대 안 붙음. 폰 핫스팟으로 먼저 성공시킨 뒤 사내망 시도.

## 7. IP 스캔으로 Pi 찾아 접속 (`pak.local` 안 될 때)

첫 부팅에 인터넷이 안 되면 `avahi-daemon` 설치가 실패해서 `pak.local`이 **원래 안 풀림**. WiFi가 붙었어도 마찬가지 → IP로 찾는다.

```bash
# 1) 내 PC 대역 확인
ip -4 -o addr show | grep -v " lo "
# 예: wlp128s20f3  inet 10.2.12.77/24  → 대역 10.2.12.0/24

# 2) 22번 포트(SSH) 열린 기기 전부 찾기
nmap -p22 --open -T4 -n 10.2.12.0/24 -oG - | grep "22/open"

# 3) 각 IP의 SSH 버전 + MAC 확인
for ip in $(nmap -p22 --open -T4 -n 10.2.12.0/24 -oG - | awk '/22\/open/{print $2}'); do
  printf "%-15s %-45s %s\n" "$ip" "$(timeout 3 nc $ip 22 </dev/null | head -1 | tr -d '\r')" \
    "$(ip neigh | awk -v i=$ip '$1==i{print $5}')"
done
```

Pi 판별 기준:

- MAC 앞자리 `2c:cf:67`, `d8:3a:dd`, `dc:a6:32`, `e4:5f:01`, `b8:27:eb` = Raspberry Pi
- SSH 배너에 26.04 버전(`OpenSSH_10.2p1 Ubuntu-2ubuntu…`)이 나옴. 24.04는 `OpenSSH_9.6p1 Ubuntu-3ubuntu13…`
- MAC만 알면 바로: `ip neigh | grep -i 2c:cf:67` (스캔 직후에 해야 ARP 테이블에 있음)

실측 결과 (2026-10-27, KANT_Teamsparta):

| IP | SSH 배너 | MAC | |
|---|---|---|---|
| **10.2.12.198** | OpenSSH_10.2p1 Ubuntu-2ubuntu3.5 | 2c:cf:67:47:1e:a1 | **Pi (pak)** |
| 10.2.12.79 / 110 / 123 / 134 / 136 | OpenSSH_9.6p1 Ubuntu-3ubuntu13.x | — | 다른 기기 |
| 10.2.12.115 | OpenSSH_10.3 | — | 다른 기기 |

```bash
ssh pak@10.2.12.198
# 접속 후 mDNS 설치 → 다음부터 pak.local 가능
sudo apt update && sudo apt install -y avahi-daemon
```

> DHCP라서 IP는 바뀔 수 있음. 바뀌면 위 스캔을 다시 하고 `~/.ssh/config`의 `HostName`만 고치면 됨.

### 7-1. 첫 부팅 때 WiFi가 안 붙었던 원인 (26.04 실측)

첫 부팅 로그에서 `wlan0`가 Up=False였고, 실제 시스템에서는 `systemd-networkd`가 한 번도 실행되지 않음. `netplan-configure.service`(05:44:12)가 cloud-init이 `/etc/netplan/50-cloud-init.yaml`을 쓰기(05:44:28) **전에** 실행돼서 아무것도 구성하지 못한 것으로 보임(로그로 추정).
**한 번 더 재부팅하면 정상적으로 붙음.** 모니터 없이 쓰려면 첫 부팅 후 3분 정도 기다렸다가 전원을 뽑고 다시 켜는 게 좋음.

## 8. SSH 키 등록 (비밀번호 없이 접속)

```bash
# 1) 키 없으면 생성 (이미 ~/.ssh/id_ed25519 있으면 생략)
ls ~/.ssh/id_ed25519.pub || ssh-keygen -t ed25519 -C "pank@legion"

# 2) 별칭 등록: ~/.ssh/config
cat >> ~/.ssh/config <<'EOF'
Host pak
    HostName 10.2.12.198
    User pak
    IdentityFile ~/.ssh/id_ed25519
    IdentitiesOnly yes
EOF
chmod 600 ~/.ssh/config

# 3) 공개키를 Pi에 복사 (이때만 Pi 비밀번호 1회 입력)
ssh-copy-id -i ~/.ssh/id_ed25519.pub pak                  # ~/.ssh/config 별칭 사용
ssh-copy-id -i ~/.ssh/id_ed25519.pub pak@10.2.12.198      # 별칭 없이 IP로 직접
```

> ⚠️ IP만 쓰면(`ssh-copy-id ... 10.2.12.198`) **PC 사용자명(`pank`)으로 접속**함 → `pank@10.2.12.198's password:` 가 뜨고 비번 맞아도 `Permission denied`.
> 반드시 `pak@IP` 형태로 Pi 사용자명을 붙일 것. 프롬프트가 `pak@10.2.12.198's password:` 인지 확인.

```bash
# 4) 확인: 비번 없이 붙으면 성공
ssh pak
ssh -o BatchMode=yes pak hostname   # 비번 물으면 실패, "pak" 출력되면 성공
```

`ssh-copy-id`는 Pi의 `~/.ssh/authorized_keys`에 공개키를 추가하고 권한(700/600)까지 맞춰 줌. 수동으로 하려면:

```bash
cat ~/.ssh/id_ed25519.pub | ssh pak@10.2.12.198 'mkdir -p ~/.ssh && chmod 700 ~/.ssh && cat >> ~/.ssh/authorized_keys && chmod 600 ~/.ssh/authorized_keys'
```

(선택) 키 접속이 되는 걸 확인한 뒤 Pi에서 비밀번호 로그인을 끄면 더 안전함:

```bash
ssh pak 'echo "PasswordAuthentication no" | sudo tee /etc/ssh/sshd_config.d/99-nopass.conf && sudo systemctl restart ssh'
```

## 9. `pak.local` 활성화 (mDNS / avahi)

`pak.local` 이름 해석은 **Pi에 `avahi-daemon`이 돌고 있어야** 됨. 26.04 첫 부팅에서 네트워크가 안 붙어 cloud-init의 `packages: [avahi-daemon]` 설치가 실패 → 아예 설치 안 된 상태였음(실측).

### 9-1. 현재 상태 확인

```bash
# Pi 쪽 (키 등록 후 비번 없이 실행됨)
ssh pak 'systemctl is-active avahi-daemon; dpkg -l avahi-daemon | tail -1'
# inactive / "no packages found" → 미설치

# PC 쪽 (이미 정상: libnss-mdns 설치 + avahi active)
grep hosts /etc/nsswitch.conf        # mdns4_minimal 있어야 함
systemctl is-active avahi-daemon     # active
```

### 9-2. Pi에 설치 & 활성화

Pi의 `sudo`는 비번을 물으므로 `-t`(터미널 할당)를 붙여 실행. 비번 프롬프트가 뜨면 Pi(`pak`) 비번 입력.

```bash
ssh -t pak 'sudo apt update && sudo apt install -y avahi-daemon libnss-mdns && sudo systemctl enable --now avahi-daemon && systemctl is-active avahi-daemon'
```

또는 접속해서 직접:

```bash
ssh pak
sudo apt update
sudo apt install -y avahi-daemon libnss-mdns
sudo systemctl enable --now avahi-daemon
systemctl status avahi-daemon --no-pager    # active (running), "Server startup complete. Host name is pak.local"
exit
```

### 9-3. PC에서 확인

```bash
avahi-resolve -n pak.local      # pak.local  10.2.12.198
getent hosts pak.local
ping -c3 pak.local
ssh pak@pak.local
```

IP가 바뀌어도 이름으로 찾도록 `~/.ssh/config`의 `HostName`을 바꿔두면 편함:

```
Host pak
    HostName pak.local
    User pak
    IdentityFile ~/.ssh/id_ed25519
    IdentitiesOnly yes
```

### 9-4. 그래도 `pak.local`이 안 풀리면

- `avahi-resolve`가 타임아웃 → 네트워크가 멀티캐스트(mDNS, UDP 5353)를 막는 경우. 사내/게스트 WiFi(AP Isolation)에서 흔함. 이 경우는 IP 접속(7장)만 가능.
- `pak-2.local`로 잡힘 → 같은 망에 hostname `pak`이 또 있음. 한 대 hostname 변경: `sudo hostnamectl set-hostname pak2`
- Pi 방화벽 켰다면: `sudo ufw allow 5353/udp`

> Pi를 다시 구우면 호스트 키가 바뀌어서 `REMOTE HOST IDENTIFICATION HAS CHANGED` 경고가 뜸 → `ssh-keygen -R 10.2.12.198` 후 다시 접속.

---
작성: 2026-10-27 / 호스트: Ubuntu (pank-Legion-Pro) / 대상: Pi 4B hostname `pak`
