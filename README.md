# 🛩️ Trokrilna autorotacijska UAV platforma 

> **Sveučilište u Zagrebu, FER — LARICS/ZARI**
> **Autori:** Toni Vuković, Teo Putarek, Marko Šimić
> **Projekt:** Dizajn i razvoj trokrilne autorotacijske UAV platforme

---

## ✨ Sažetak

Ovaj repozitorij sadrži **cjelokupan kod, 3D modele, sheme i dokumentaciju** za trokrilnu UAV platformu koja koristi **autorotaciju** za sigurno spuštanje pri gubitku pogona. Sustav je temeljen na **Arduino** mikrokontroleru, **XBee DigiMesh S2C** bežičnim modulima i **Python** zemaljskoj stanici s **joystick** upravljanjem, **watchdog/failsafe** logikom i obradom **IMU** telemetrije.

---

## 🚀 Značajke

* Trokrilna UAV geometrija s rotorima na vrhovima krila (120°)
* Autorotacijski dizajn za **kontrolirano spuštanje** bez aktivnog pogona
* **XBee DigiMesh S2C** transparent mode (115200 bps) za dvosmjernu telemetriju
* Zemaljska stanica u **Pythonu**: čitanje **Logitech Attack 3** joysticka, slanje komandi, vizualizacija
* **Watchdog** na Python i Arduino strani + **failsafe** (gašenje motora nakon \~300 ms bez poruke)
* Arduino firmware: PWM upravljanje **brushed DC** motorima (pinovi 9/10/11), čitanje **LSM9DS1TR** IMU-a
* Prikupljanje telemetrije u **CSV**; skripte za grafove i analizu

---

## 🔩 Hardware

* **XBee DigiMesh S2C** (wire antenna), **transparent mode**
* **Level shifter** između 5 V (Arduino) i 3.3 V (XBee)
* **Brushed DC** motori + propeleri
* **LiPo 2S 7.4 V, \~600 mAh** + **DC-DC 7.4→5 V**
* **Arduino** (UNO/Nano) — PWM pinovi **9/10/11** za motore
* **IMU LSM9DS1TR** (akcelerometar/žiroskop/magnetometar)
* **Joystick Logitech Attack 3**

---

## 🧊 3D modeli

* **Krila** u 2 dijela (print na manjim 3D printerima)
* **Centralni konektor** (cilindar s utorima na 120°)
* **Tijelo** (lagana plastična boca) + **3D print kupola** (navoj)
* **Nosači motora** i kanali za žice duž krila

---

## 💡 Firmware (Arduino)

* Serijska veza: **115200 bps**
* PWM upravljanje na pinovima **9/10/11** (`analogWrite` 0–255)
* Čitanje IMU-a (LSM9DS1TR), filtriranje: jednostavni **low‑pass**
* **Watchdog**: gašenje motora nakon \~300 ms bez serijske poruke

---

## 🐍 Python aplikacije

* `ground/joystick_send.py` — čitanje joysticka, slanje komandi (x/y/z + tipke)
* `tools/plot_*` — vizualizacija yaw/kutne brzine i CSV parseri
* Watchdog thread koji periodički obnavlja zadnju poruku

---

## 🧱 Sigurnost i regulativa

* Failsafe gašenje motora pri gubitku veze
* Uskladiti upotrebu s lokalnim UAS propisima (kategorija "otvorena" itd.)

