1. Ollama를 LAN에 열기
   sudo mkdir -p /etc/systemd/system/ollama.service.d
   sudo tee /etc/systemd/system/ollama.service.d/override.conf >/dev/null <<'EOF'
   [Service]
   Environment="OLLAMA_HOST=0.0.0.0:11434"
   EOF
   sudo systemctl daemon-reload
   sudo systemctl restart ollama
2. ufw에서 11434 허용
   sudo ufw allow 11434/tcp
   sudo ufw reload
3. 확인
   PC에서:
   ss -ltnp | grep 11434
   0.0.0.0:11434로 보여야 합니다.