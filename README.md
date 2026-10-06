# ROS 2 Cart-Pole Control

Ters sarkaç (cart-pole) simülasyonu ve **LQR / PID** denge kontrolü, **ROS 2 (Jazzy)** düğümleri olarak.
Önceki [cartpole-balance-control](../cartpole-balance-control) projesindeki fizik ve denetleyiciler
burada üç ayrı düğüme bölündü: simülatör, denetleyici ve kayıt düğümü birbirleriyle topic ve servislerle konuşuyor.

## Mimari

```
                 /cartpole/state  (Float64MultiArray: x, x_dot, theta, theta_dot)
   ┌──────────┐ ───────────────────────────────►┌──────────────────┐
   │ sim_node │                                  │ controller_node  │
   └──────────┘ ◄───────────────────────────────└──────────────────┘
        ▲        /cartpole/force  (Float64: N)         (LQR veya PID)
        │ servisler: /cartpole/push, /cartpole/reset
        │
   ┌─────────────┐   state ve force topic'lerini dinler, CSV'ye yazar
   │ logger_node │
   └─────────────┘
```

| Düğüm | Görevi |
|---|---|
| `sim_node` | Doğrusal olmayan fiziği RK4 ile 200 Hz entegre eder, durumu yayınlar. `/cartpole/push` ile dış itme, `/cartpole/reset` ile sıfırlama servisleri sunar |
| `controller_node` | Durumu dinler, `controller_type` parametresine göre LQR veya PID kuvveti hesaplar, ±30 N ile sınırlar |
| `logger_node` | Durum ve kuvveti CSV'ye kaydeder |

Fizik ve denetleyici kodu (`dynamics.py`, `controllers.py`) ROS'a bağımlı değil, bu yüzden `pytest` ile ROS olmadan test ediliyor.

## Çalıştırma (GitHub Codespaces)

Repoda `.devcontainer/` var, Codespace açılırken ROS 2 Jazzy ortamı otomatik kurulur (ilk açılış birkaç dakika sürer).

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch cartpole_ros cartpole.launch.py
```

PID ile başlatmak için: `ros2 launch cartpole_ros cartpole.launch.py controller:=pid`

Başka bir terminalde sistemi incele ve müdahale et:

```bash
ros2 node list
ros2 topic list
ros2 topic echo /cartpole/state
ros2 topic hz /cartpole/state
ros2 service call /cartpole/push std_srvs/srv/Trigger   # 15 N, 0.2 s dış itme
ros2 service call /cartpole/reset std_srvs/srv/Trigger  # başarısızlıktan sonra sıfırla
ros2 param get /cartpole_controller controller_type
```

Kaydı grafiğe çevirmek için (launch'u Ctrl+C ile durdurduktan sonra):

```bash
python3 tools/plot_log.py /tmp/cartpole_log.csv results/lqr_run.png
```

## Testler

```bash
cd src/cartpole_ros && python3 -m pytest -q
```

8 test: denge noktası, doğrusallaştırmanın doğruluğu, açık çevrimin kararsızlığı, LQR'ın kapalı çevrim kararlılığı, 10° eğimden toparlanma ve sadece açı geri beslemeli PID'in araba kaymasıyla raydan çıkması.

## Sonuçlar

> `results/` altındaki grafikleri kendi çalıştırmandan ekle ve buraya birkaç cümle yaz:
> LQR ve PID'in açı ve araba konumu davranışı, itme sonrası toparlanma süresi.

![LQR kaydı](results/lqr_run.png)

## Sınırlılıklar
- Tam durum ölçümü varsayıldı (sensör gürültüsü ve durum kestirimi yok)
- Simülatör tek makinede gerçek zamansız (wall-clock) timer ile çalışıyor; `use_sim_time` ve `ros2 bag` entegrasyonu yok
- Kontrol döngüsü simülatörün yayın hızına bağlı (bir adım gecikme)
- Sonraki adımlar: Kalman filtresi düğümü, `ros2 bag` ile kayıt/oynatma, özel mesaj tipleri, Gazebo
