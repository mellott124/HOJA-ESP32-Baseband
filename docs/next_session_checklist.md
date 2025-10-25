# Next-Session Checklist

## 🧩 1. Code polish and safety
- [ ] Add explicit LED blink patterns  
  • **Fast blink (200 ms)** → reconnect in progress  
  • **Slow blink (500 ms)** → fresh pairing  
- [ ] Add a small debounce delay to SYNC input (≈ 50 ms) to filter contact bounce.  
- [ ] Double-check all task handles are `NULL` after deletion to prevent double signals.

## ⚡ 2. Power and sleep optimization
- [ ] Suspend `_switch_bt_task_standard` when disconnected > 30 s to cut idle current.  
- [ ] Resume task when GAP reconnect/pairing events occur.  
- [ ] Optional: enter light-sleep until SYNC press wakes the MCU.

## 🧠 3. Reliability & diagnostics
- [ ] Add a watchdog counter for “failed reconnect” attempts (for field debugging).  
- [ ] Add an `ESP_LOGI()` summary on boot showing paired host, last disconnect reason, and battery voltage (if applicable).  
- [ ] Confirm reconnects still succeed after ~10 sleep/wake cycles.

## 🧰 4. Versioning & maintenance
- [ ] Tag this working state clearly  
  ```bash
  git tag -a stable_bt_reconnect_2025_10_24 -m "Full BT stability verified"
  git push origin main --tags
  ```
- [ ] Create a `dev/next` branch for any new experiments (LED, sleep, etc.)  
  → keeps `main` pristine and rollback-safe.

## 🚀 5. Future features (optional stretch goals)
- [ ] Add controller battery reporting to Switch (simulate Joy-Con battery byte).  
- [ ] Implement vibration pattern profiles.  
- [ ] Add EEPROM or NVS “last reconnect reason” for field troubleshooting.
