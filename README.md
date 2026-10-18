# ardupilot-mcp

[![CI](https://github.com/furkanisikay/ardupilot-mcp/actions/workflows/ci.yml/badge.svg)](https://github.com/furkanisikay/ardupilot-mcp/actions/workflows/ci.yml)
[![PyPI](https://img.shields.io/pypi/v/ardupilot-mcp.svg)](https://pypi.org/project/ardupilot-mcp/)
[![Python](https://img.shields.io/pypi/pyversions/ardupilot-mcp.svg)](https://pypi.org/project/ardupilot-mcp/)
[![License: AGPL-3.0](https://img.shields.io/badge/license-AGPL--3.0-blue.svg)](LICENSE)

Türkçe · [English](README.en.md)

<!-- mcp-name: io.github.furkanisikay/ardupilot-mcp -->

ArduPilot uçuş loglarını teşhis eden bir MCP sunucusu. Claude Desktop, Cursor ya da VS Code gibi bir yapay zekâ istemcisine eklersin, `.bin` veya `.tlog` dosyanı verirsin; istemci uçuşta ne olduğunu sade bir dille anlatır: titreşim, düşük batarya, GPS kaybı, motor kaybı, hatalı ayar.

Bulguları deterministik kontroller üretir. Her eşiğin kaynağı [docs/SOURCES.md](docs/SOURCES.md)'de yazılıdır; model bu bulguları yalnızca açıklar. Sunucu yalnızca yerel dosyaları okur, ağa bağlanmaz, araca komut göndermez ve parametre yazmaz.

## Hızlı başlangıç

Claude Desktop ile:

1. [uv](https://docs.astral.sh/uv/getting-started/installation/)'yi kur.
2. Claude Desktop'ta Settings → Developer → Edit Config ile şunu ekle:
   ```json
   { "mcpServers": { "ardupilot-mcp": { "command": "uvx", "args": ["ardupilot-mcp"] } } }
   ```
3. Claude Desktop'tan tamamen çık ve yeniden aç.
4. Sor: *"C:\loglar\00000042.BIN logunu analiz et, neden düştü?"*

Claude Code için tek satır yeterli: `claude mcp add ardupilot-mcp -- uvx ardupilot-mcp`. Cursor, VS Code, Windsurf, Cline, Zed, LM Studio ve diğer istemcilerin ayarları, sorun giderme ve istemcisiz deneme [docs/clients.md](docs/clients.md)'de.

## Hangi log?

- **`.bin`** uçuş kontrolcüsünün kendi kaydıdır ve en ayrıntılı teşhisi verir. SD karttaki `APM/LOGS/` klasöründedir; Mission Planner'da DataFlash Logs → Download DataFlash Log Via Mavlink ile de indirilir.
- **`.tlog`** yer istasyonunun kaydettiği telemetridir. Saniyede birkaç örnek ve yalnızca birincil sensör taşır, bu yüzden teşhis daha kabadır. Telsiz kopmasını araç arızası sanabilecek kontroller bu dosyada çalışmaz.

## Örnek

`python examples/analyze.py demo.bin` çıktısından bir bölüm. İstemci aynı bulguları senin dilinde anlatır.

<!-- gen:example -->
```text
First anomaly while armed: Elevated VibeZ vibration @ 9s.
6 critical, 2 warning, 3 info finding(s).
[CRITICAL] vibration @ 9s: Elevated VibeZ vibration
           VibeZ vibration peaks at 72 m/s^2 (>60); 20% of samples exceed 60 m/s^2.
[CRITICAL] attitude  @ 20s: Attitude tracking failure (roll)
           Roll failed to track the demand for 3.0 s, error up to 35 deg (sustained >25 deg).
[CRITICAL] power     @ 25s: Battery reached critical voltage
           Pack voltage fell to 13.32 V, at or below the critical-voltage level (13.50 V, configured BATT_*_VOLT failsafe).
```
<!-- /gen:example -->

## Araçlar

Bütün araçlar salt okunurdur.

| Araç | Ne işe yarar |
|---|---|
| `analyze_log` | Bütün kontrolleri çalıştırır; önem sırasına dizilmiş bulguları resmi ArduPilot doküman linkleriyle döndürür. |
| `incident_timeline` | İlk anomaliyi ve çarpma anını bulur, olayları zaman sırasına koyar. Neden-sonuç iddia etmez. |
| `forum_report` | discuss.ardupilot.org'a yapıştırılacak Markdown yanıtı hazırlar. GPS konumu içermez. |
| `compare_logs` | İki uçuş arasında değişen parametreleri, firmware'i ve metrikleri yan yana koyar. |
| `log_summary` | Araç, firmware, süre, uçuş modları ve log bütünlüğü. |
| `vehicle_profile` | Frame, motor sayısı, batarya, hover gazı ve güç marjı. |
| `list_events` | `ERR`, `MODE`, `EV` ve `MSG` kayıtlarının zaman çizelgesi. |
| `query_timeseries` | Bir bulgunun arkasındaki ham seri; çoklu sensörlerde instance seçilebilir. |
| `get_params` | Logdaki parametreler ve o firmware sürümünde belgelenmiş aralık, birim ve değerler. |
| `load_param_file` | Dışa aktarılmış `.param` dosyasını okur. |
| `recommend_tuning` | Harmonik notch, PID ve autotune önerisi. Hiçbir şeyi uygulamaz. |
| `list_checks` | Kayıtlı kontrollerin listesi. |

Araçların argümanları ve <!-- gen:checks -->18<!-- /gen:checks --> kontrolün tamamı [docs/reference.md](docs/reference.md)'de. Bu dosyayı `scripts/gen_docs.py` koddan üretir, CI da güncel olup olmadığını denetler.

## Nasıl çalışır

Parser logu sütunlu bir `FlightLog` modeline çevirir. Kayıtlı kontroller bu modeli okur: verisi logda olmayan kontrol atlanır, araç tipine uymayan çalışmaz. Aynı olayın ERR kaydı, failsafe mod değişikliği ve firmware mesajı tek bulguda birleşir. Çarpmadan sonra başlayan bulgular listenin sonuna iner ve raporun başında çarpmadan önceki ilk anomali kalır.

4.x firmware her IMU, GPS, pusula, batarya ve EKF çekirdeğini tek mesaj adı altında yazar. Kontroller her birini ayrı değerlendirir. 4.1 öncesi loglarda ayrı adla yazılan ikincil sensörler de aynı yapıya çevrilir.

Sunucu MCP Python SDK 2 üzerinde çalışır. Hem 2026-07-28 protokol sürümünü hem de bugün çoğu masaüstü istemcinin kullandığı eski el sıkışmayı (2024-11-05 ile 2025-11-25 arası) konuşur.

## Doğruluk ve sınırlar

Motor, forumdaki <!-- gen:bench-logs -->49<!-- /gen:bench-logs --> gerçek kaza logu üzerinde ölçülür; logların sebebini o logu inceleyen bir geliştirici ya da deneyimli bir kullanıcı yazmıştır. Güncel skorlar ve yöntem [bench/README.md](bench/README.md)'de.

- Pusula yön hatasından kaynaklanan dairesel sürüklenme (toilet-bowl) henüz tespit edilmiyor.
- Format tanımları bozulmuş bir log, kısmi sonuç yerine açık bir hata verir.
- `.tlog` dosyasında ERR kaydı, batch sampler, hızlı IMU ve mod değişikliği nedeni yoktur.
- Bulgular karar desteğidir. Uçuşa elverişlilik kararı vermez.

## Diğer araçlar

[UAV Log Viewer](https://plot.ardupilot.org/) ve Mission Planner'ın [Auto Analysis](https://ardupilot.org/planner/docs/common-diagnosing-problems-using-logs.html)'i veriyi gösterir, yorumu sana bırakır. ardupilot-mcp yorumu yapar ve takip sorularını cevaplar. Bir sinyali grafikte görmek ya da uçuşu 3B izlemek için UAV Log Viewer daha iyidir.

## Katkı

[CONTRIBUTING.md](.github/CONTRIBUTING.md) geliştirme ortamını, yeni kontrol eklemeyi ve PR sürecini anlatır. Yapay zekâ asistanlarının uyduğu kurallar [AGENTS.md](AGENTS.md)'de. Güvenlik açıklarını [SECURITY.md](.github/SECURITY.md)'deki yoldan bildir. Değişiklikler [CHANGELOG.md](CHANGELOG.md)'de.

## Lisans

[GNU AGPL-3.0](LICENSE). Kodu değiştirip ağ üzerinden hizmet olarak sunan, değiştirdiği kaynak kodu o hizmetin kullanıcılarına açmak zorundadır. AGPL koşulları işine uymuyorsa ticari lisans için contact@furkanisikay.com adresine yaz. PyPI'da yayımlanan 0.1.3 ve önceki paketler MIT lisanslıdır ve öyle kalır; bu değişiklikten sonraki kaynak kod ve sonraki sürümler AGPL-3.0 lisanslıdır.

`ardupilot_mcp/data/params/` altındaki veri, ArduPilot'un GPLv3 lisanslı parametre tanımlarından (`apm.pdef.xml`) çıkarılmış olgulardan oluşur: aralık, birim, değer kodları ve bitmask bitleri. Açıklama metni kopyalanmaz.
