# ardupilot-mcp

[![CI](https://github.com/furkanisikay/ardupilot-mcp/actions/workflows/ci.yml/badge.svg)](https://github.com/furkanisikay/ardupilot-mcp/actions/workflows/ci.yml)
[![PyPI](https://img.shields.io/pypi/v/ardupilot-mcp.svg)](https://pypi.org/project/ardupilot-mcp/)
[![Python](https://img.shields.io/pypi/pyversions/ardupilot-mcp.svg)](https://pypi.org/project/ardupilot-mcp/)
[![License: MIT](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

🇹🇷 Türkçe · [🇬🇧 English](README.en.md)

<!-- mcp-name: io.github.furkanisikay/ardupilot-mcp -->

**Drone'un mu düştü ya da kötü mü uçtu?** ArduPilot uçuş kontrolcüsü her uçuşta bir kara-kutu dosyası tutar — `.bin` uçuş logu. Bu araç o dosyayı okur ve bir yapay zekâ sohbet uygulamasının uçuşta ne olduğunu **düz dille** anlatmasını sağlar: düşük batarya, aşırı titreşim, GPS kaybı, kablo arızası, kötü ayar… Tahminle değil, deterministik bir kontrol motoruyla.

**Nasıl bir şey bu?** Kendi penceresi/uygulaması yok. Bir yapay zekâ sohbet uygulamasına takılır — **Claude Desktop, Cursor, VS Code** gibi (bunlara "MCP istemcisi" denir). Önce o uygulamalardan birini kurarsın, bu aracı ona eklersin, sonra düz dille sorarsın. (MCP = Model Context Protocol; bu uygulamaların araçlarla konuştuğu standart "fiş".)

## Neden bunu kullanayım?

Mission Planner'ın [Auto Analysis](https://ardupilot.org/planner/docs/common-diagnosing-problems-using-logs.html)'i ve [UAV Log Viewer](https://plot.ardupilot.org/) zaten var ve iyiler — ama farklı bir iş yapıyorlar. Kısaca: o araçlar **veriyi gösterir, yorumu sen yaparsın**; bu araç **veriyi yorumlar, düz dille anlatır ve sorularını cevaplar.**

| | MP Auto Analysis | UAV Log Viewer | **ardupilot-mcp** |
|---|:---:|:---:|:---:|
| Çıktı | sabit geç/kal listesi | grafik + 3B uçuş tekrarı | düz dille açıklama + sohbet |
| "Neden düştü?" diye sorup gerekçeli cevap | — | — | **✓** |
| Doğal dille takip sorusu ("15–30 sn titreşim?") | — | — | **✓** |
| Konfig/kurulum hataları (param denetimi, kalibrasyon, kablo, pre-arm) | kısmen | — | **✓** |
| Fiziksel muhakeme (güç marjı, itki/ağırlık) | — | — | **✓** |
| Her bulguda resmi ArduPilot doküman linki | — | — | **✓** |
| Grafik / 3B harita görselleştirme | — | **✓** | — |
| Anında, kurulum yok | **✓** (MP'de hazır) | **✓** (web) | AI istemcisi + uv gerekir |

**Diğerlerini ne zaman kullan:** bir sinyali grafikte görmek veya uçuş yolunu 3B izlemek için → **UAV Log Viewer**. Elinde Mission Planner açıkken hızlı bir geç/kal için → **MP Auto Analysis**. Bu araç onların yerini almaz; **"neden böyle oldu?"yu açıklaması ve takip sorularını cevaplaması**, kararı deterministik bir motora + resmi dokümana dayandırması, ve uçuş sinyallerinin yanı sıra **konfig/kurulum ve fiziksel** tarafı da kapsaması için var. Ayrıca tamamen offline ve salt-okunur çalışır.

## Hızlı başlangıç (5 dakika)

En kısa yol — Claude Desktop ile:

1. **Claude Desktop'ı kur:** <https://claude.ai/download>
2. **uv'yi kur** — bu aracı senin yerine indirip çalıştıran küçük, ücretsiz bir program (`uvx` onunla gelir):
   - Windows (PowerShell): `powershell -c "irm https://astral.sh/uv/install.ps1 | iex"`
   - macOS / Linux: `curl -LsSf https://astral.sh/uv/install.sh | sh`
   - Dokümanı: <https://docs.astral.sh/uv/getting-started/installation/> · Çalıştı mı: `uv --version` bir sürüm yazmalı.
3. **Claude Desktop'ta** Settings → Developer → Edit Config; şunu yapıştır ve kaydet:
   ```json
   { "mcpServers": { "ardupilot-mcp": { "command": "uvx", "args": ["ardupilot-mcp"] } } }
   ```
   Sonra Claude Desktop'ı **tamamen kapatıp yeniden aç** — pencereyi kapatmak yetmez; sistem tepsisinden de çık.
4. **Bir `.bin` logu edin** (aşağıdaki *Uçuş logunu nereden bulurum?*). Elinde yoksa *AI istemcisi olmadan dene* ile demo üret.
5. **Sohbet kutusuna yaz:** *"C:\loglar\ucus.bin logunu analiz et — neden düştü?"* (yolu kendi dosyanla değiştir).

uv yerine pip kullanmak ya da başka bir uygulama (Cursor, VS Code, Cline, LM Studio…) eklemek için → [İstemcine ekle](#istemcine-ekle).

## Uçuş logunu (`.bin`) nereden bulurum?

Uçuş kontrolcün her uçuş için bir `.bin` DataFlash logu kaydeder. Bilgisayarına almak için:

- **En kolayı — Mission Planner:** USB ile bağlan → **Flight Data** → **DataFlash Logs** → **Download DataFlash Log Via Mavlink** → uçuşu seç → kaydet (ör. `C:\loglar\`). QGroundControl da olur.
- **Ya da SD karttan:** kontrolcünün SD kartında genelde `APM/LOGS` veya `/LOGS` klasörü; dosya adı `00000042.BIN` gibi görünür.

Tam dosya yolunu not et — yapay zekâya vereceğin şey o. Detay: [ArduPilot — Downloading and Analyzing Data Logs](https://ardupilot.org/copter/docs/common-downloading-and-analyzing-data-logs-in-mission-planner.html).

## AI istemcisi olmadan dene

Bir AI uygulaması kurmadan, motorun çalıştığını görmek istersen kaynağı indir (örnek scriptler pip paketinde gelmez, repoyu klonlaman gerekir):

```bash
git clone https://github.com/furkanisikay/ardupilot-mcp
cd ardupilot-mcp
pip install -e .
python examples/generate_demo_log.py demo.bin   # gerçekçi bir kaza logu üretir
python examples/analyze.py demo.bin              # tam teşhisi terminale yazar
```

Hazır olunca `demo.bin` yerine kendi `.bin` dosyanı ver.

## Cevap neye benziyor?

Yukarıdaki `analyze.py demo.bin` çıktısından (kısaltıldı). Bir AI istemcisinde aynı bulgular **senin dilinde**, sohbet havasında anlatılır:

```
ArduCopter V4.5.7 — 30 sn uçuş — 8 kritik, 2 uyarı, 3 bilgi

[KRİTİK] titreşim @ 9s  : VibeZ titreşimi 72 m/s² zirve yaptı (>60); örneklerin %20'si eşiği aştı.
[KRİTİK] attitude @ 20s : Roll komutu 3 sn takip edilemedi, hata 35°'ye çıktı — kontrol kaybı / mekanik arıza.
[KRİTİK] gps      @ 22s : GPS 3D fix'i 22–24 sn arasında kayboldu — konum desteği gitti.
[KRİTİK] güç      @ 30s : Paket voltajı 13.22 V'a düştü (kritik 13.50 V failsafe) — acil iniş gerek.
...
TUNING (yalnız tavsiye): 80 Hz harmonik notch öner.
```

Her bulgu ilgili resmi ArduPilot doküman linkini ve somut sayıları (zaman, değer, eşik) taşır.

## İstemcine ekle

Tek tıkla:

[![Add to Cursor](https://img.shields.io/badge/Add_to-Cursor-000000?style=flat-square&logo=cursor&logoColor=white)](cursor://anysphere.cursor-deeplink/mcp/install?name=ardupilot-mcp&config=eyJjb21tYW5kIjoidXZ4IiwiYXJncyI6WyJhcmR1cGlsb3QtbWNwIl19)
[![Install in VS Code](https://img.shields.io/badge/VS_Code-Install_Server-0098FF?style=flat-square&logo=visualstudiocode&logoColor=white)](https://insiders.vscode.dev/redirect/mcp/install?name=ardupilot-mcp&config=%7B%22command%22%3A%22uvx%22%2C%22args%22%3A%5B%22ardupilot-mcp%22%5D%7D)
[![Install in VS Code Insiders](https://img.shields.io/badge/VS_Code_Insiders-Install_Server-24bfa5?style=flat-square&logo=visualstudiocode&logoColor=white)](https://insiders.vscode.dev/redirect/mcp/install?name=ardupilot-mcp&config=%7B%22command%22%3A%22uvx%22%2C%22args%22%3A%5B%22ardupilot-mcp%22%5D%7D&quality=insiders)
[![Add to LM Studio](https://files.lmstudio.ai/deeplink/mcp-install-light.svg)](https://lmstudio.ai/install-mcp?name=ardupilot-mcp&config=eyJhcmR1cGlsb3QtbWNwIjp7ImNvbW1hbmQiOiJ1dngiLCJhcmdzIjpbImFyZHVwaWxvdC1tY3AiXX19)

**Kurulum (bir kez).** Aşağıdaki config'lerin hepsi `uvx ardupilot-mcp` komutunu kullanır.

- **Önerilen — uv:** yukarıdaki *Hızlı başlangıç* adım 2'deki tek satırla kur. uv kuruluysa config'ler olduğu gibi çalışır; ayrıca `pip install` gerekmez.
- **Alternatif — pip:** `pip install ardupilot-mcp` yaptıysan, aşağıdaki config'lerde `"command": "uvx", "args": ["ardupilot-mcp"]` yerine `"command": "ardupilot-mcp"` yaz (args'sız). İkisi de çalışır; ikisini karıştırma.

Python da gerekir (3.11+). Kontrol: terminalde `python --version` 3.11 veya üstünü yazmalı. Yoksa <https://python.org/downloads>'dan kur (Windows'ta kurulumda **"Add Python to PATH"**i işaretle). uv yolunu kullanırsan uv uygun bir Python'u kendi indirebilir, ayrıca kurman gerekmeyebilir.

> **Önemli:** Config'i ekledikten/değiştirdikten sonra uygulamayı **yeniden yükle**. Dosya-tabanlı uygulamalarda (Claude Desktop, Cursor, VS Code, Windsurf, LM Studio, Zed) **tamamen kapatıp aç** — pencereyi kapatmak yetmez, tepside/menü çubuğunda açık kalır. Sonra araç listesini kontrol et.

<details>
<summary><b>Claude Code</b></summary>

```bash
claude mcp add ardupilot-mcp -- uvx ardupilot-mcp
```

Tüm projelerde kullanmak için `--scope user`, takımla paylaşmak için `--scope project` (repoya `.mcp.json` yazar). Doğrula: `claude mcp list`.

Ya da eklenti olarak (MCP sunucusu + `/diagnose` komutu birlikte gelir):

```
/plugin marketplace add furkanisikay/ardupilot-mcp
/plugin install ardupilot-mcp@ardupilot-tools
```
</details>

<details>
<summary><b>Claude Desktop</b></summary>

Settings → Developer → Edit Config ile `claude_desktop_config.json`'a ekle, sonra Claude Desktop'ı tamamen kapatıp yeniden aç:

```json
{
  "mcpServers": {
    "ardupilot-mcp": { "command": "uvx", "args": ["ardupilot-mcp"] }
  }
}
```
</details>

<details>
<summary><b>Cursor</b></summary>

Üstteki **Add to Cursor** düğmesi; ya da `~/.cursor/mcp.json` (projeye özel: `.cursor/mcp.json`):

```json
{
  "mcpServers": {
    "ardupilot-mcp": { "command": "uvx", "args": ["ardupilot-mcp"] }
  }
}
```
</details>

<details>
<summary><b>VS Code / VS Code Insiders</b></summary>

Üstteki düğme; ya da `.vscode/mcp.json`:

```json
{
  "servers": {
    "ardupilot-mcp": { "type": "stdio", "command": "uvx", "args": ["ardupilot-mcp"] }
  }
}
```

Komut satırı: `code --add-mcp '{"name":"ardupilot-mcp","command":"uvx","args":["ardupilot-mcp"]}'`
</details>

<details>
<summary><b>Antigravity (IDE + CLI)</b></summary>

`~/.gemini/config/mcp_config.json` (Windows: `%USERPROFILE%\.gemini\config\mcp_config.json`) — IDE ile CLI aynı dosyayı paylaşır:

```json
{
  "mcpServers": {
    "ardupilot-mcp": { "command": "uvx", "args": ["ardupilot-mcp"] }
  }
}
```

IDE: agent panelindeki **...** → **MCP Servers** → **View raw config** → yapıştır → **Refresh**. CLI: `agy` çalıştırıp `/mcp` ile doğrula.
</details>

<details>
<summary><b>Windsurf</b></summary>

`~/.codeium/windsurf/mcp_config.json` (Windows: `%USERPROFILE%\.codeium\windsurf\mcp_config.json`):

```json
{
  "mcpServers": {
    "ardupilot-mcp": { "command": "uvx", "args": ["ardupilot-mcp"] }
  }
}
```

Cascade → MCP paneli → **View raw config** → yapıştır → **Refresh**.
</details>

<details>
<summary><b>Cline</b></summary>

Cline paneli → **MCP Servers** → **Configure** → `cline_mcp_settings.json`:

```json
{
  "mcpServers": {
    "ardupilot-mcp": { "command": "uvx", "args": ["ardupilot-mcp"], "disabled": false, "autoApprove": [] }
  }
}
```
</details>

<details>
<summary><b>Continue</b></summary>

Çalışma alanında `.continue/mcpServers/ardupilot-mcp.yaml`:

```yaml
name: ArduPilot MCP
version: 0.0.1
schema: v1
mcpServers:
  - name: ArduPilot MCP
    type: stdio
    command: uvx
    args:
      - ardupilot-mcp
```
</details>

<details>
<summary><b>Zed</b></summary>

`settings.json` (`context_servers` altına):

```json
{
  "context_servers": {
    "ardupilot-mcp": { "command": "uvx", "args": ["ardupilot-mcp"], "env": {} }
  }
}
```
</details>

<details>
<summary><b>Goose</b></summary>

```bash
goose configure
# Add Extension → Command-line Extension → komut: uvx ardupilot-mcp
```
</details>

<details>
<summary><b>LM Studio (açık-kaynak / yerel modeller)</b></summary>

Üstteki **Add to LM Studio** düğmesi; ya da `~/.lmstudio/mcp.json` (Windows: `%USERPROFILE%\.lmstudio\mcp.json`):

```json
{
  "mcpServers": {
    "ardupilot-mcp": { "command": "uvx", "args": ["ardupilot-mcp"] }
  }
}
```
</details>

> **Model bağımsız:** ardupilot-mcp, MCP'yi stdio üzerinden konuşur; alttaki modelden bağımsız olarak MCP destekli her istemcide çalışır — tamamen açık-kaynak / yerel modeller dahil (ör. LM Studio'da yerel GGUF modeller; Ollama veya LM Studio'ya bağlanan Continue / Cline / Goose). Anthropic/OpenAI hesabı gerektirmez; yeterli olan, istemcinin seçtiği modelin araç-çağırma (tool-calling) desteğidir.

### Çalışmıyorsa

- **Araç hiç görünmüyor / "command not found: uvx"** → uv kurulu değil veya bulunamıyor. uv'yi kur (yukarıdaki link), ya da `pip install ardupilot-mcp` yapıp config'i `"command": "ardupilot-mcp"` olarak değiştir.
- **Config'i düzenledim ama hâlâ yok** → uygulamayı **tamamen** kapatıp aç (pencere kapatmak yetmez; tepside/menü çubuğunda açık kalır).
- **Doğru dosyayı mı düzenledim?** → her istemcinin kendi bölümünde yazan tam yolu kullan; JSON'da eksik virgül/parantez olmasın.
- **"Dosya bulunamadı"** → `.bin` uzantısı dahil **tam yolu** ver; Windows'ta yolu çift tırnak içine al.
- **Motorun kendisi çalışıyor mu?** → yukarıdaki *AI istemcisi olmadan dene* ile bağımsız teyit et (`python examples/analyze.py demo.bin`).

## Araçlar (hepsi salt-okunur)

| Araç | Ne yapar |
|------|----------|
| `analyze_log(path)` | Tüm kontrol paketini çalıştırır → önem-sıralı bulgular + özet. Her bulgu **resmi ArduPilot doküman linkleri** taşır; rapor 4.x firmware için sürüm-özel parametre dokümanlarını da içerir. **Manşet araç.** |
| `log_summary(path)` | Araç, firmware, süre, mesaj sayıları, uçuş modları, max irtifa, bütünlük. |
| `vehicle_profile(path)` | **Fiziksel / mimari profil**: frame & tip, motor sayısı, batarya hücresi/kapasitesi, hover gazı, **güç marjı** ve itki/ağırlık. Fiziğe-dayalı muhakeme için. |
| `list_events(path, kinds?, start_s?, end_s?)` | Çözümlenmiş `ERR`/`MODE`/`EV`/`MSG` zaman çizelgesi. |
| `query_timeseries(path, message_type, fields, start_s?, end_s?, max_points?)` | Bir bulguya derinlemesine bakmak için seyreltilmiş sayısal seri. |
| `get_params(path, name_glob?)` | Logun PARM dökümünden parametre değerleri (ör. `INS_HNTCH_*`), toplam sayı, yorumlamak için sürüm-özel metadata linki, ve döküm eksikse bir uyarı. |
| `load_param_file(path, name_glob?)` | Kullanıcının dışa aktardığı `.param`/`.params` dosyasını okur — log dökümü kesikse veya "uçtuğu config vs şimdiki"yi karşılaştırmak için tam, otoriter config. |
| `recommend_tuning(path, area?)` | Tavsiye niteliğinde tuning (gyro FFT'sinden harmonik notch, PID, autotune). **Sadece öneri — asla uygulanmaz.** |
| `list_checks()` | Kayıtlı teşhis kontrolleri (genişletilebilir katalog). |

## Örnek kullanım senaryoları

Sunucuyu istemcine ekledikten sonra yapay zekâya düz dille sor; ilgili araçları kendisi çağırır:

- **"`C:\loglar\ucus.bin` neden düştü?"** — tüm kontrol paketini çalıştırır (`analyze_log`), önem-sıralı bulguları doküman linkleriyle açıklar.
- **"Bu araç motorlarına göre yeterince güçlü müydü?"** — `vehicle_profile` ile frame / motor / batarya / hover gazından güç marjını ve itki-ağırlık oranını çıkarır.
- **"15. ile 30. saniye arası titreşim yüksek miydi?"** — `query_timeseries` ile `VIBE` serisini çekip eşiklerle karşılaştırır.
- **"Arming neden engellendi?"** — `list_events` ile `ERR`/`MSG` zaman çizelgesini ve pre-arm uyarılarını çözer.
- **"`INS_HNTCH_*` parametrelerimi göster ve harmonik notch öner."** — `get_params` + `recommend_tuning` (yalnızca tavsiye, asla uygulanmaz).
- **"Şu `.param` dosyasını logdaki ayarlarla karşılaştır."** — `load_param_file` ile uçtuğu config'i otoriter dosyayla karşılaştırır.

**Claude Code** kullanıyorsan (Claude Desktop'tan farklı bir uygulama) **ve eklentiyi kurduysan** (yukarıdaki Claude Code bölümü — düz MCP-sunucu kurulumu `/diagnose` eklemez), tek komut yeter: **`/diagnose C:\loglar\ucus.bin`** — uçtan uca teşhisi yukarıdaki araçları sırayla kullanarak yapar. Diğer istemcilerde bunun yerine düz dille sor.

## Neden log teşhisi?

Bir ArduPilot `.bin` logunu okumak uzmanlık ister: `ERR` satırlarını filtreler, mekanik arıza için `ATT.DesRoll` ile `ATT.Roll`'u karşılaştırır, titreşim / EKF / batarya clipping'ine göz gezdirirsin. Drone'un düştüğünde "neden?" sorusunun cevabı bu logun içindedir ama okumak doktorun röntgen okuması gibidir. Bu sunucu o kontrolleri deterministik olarak çalıştırır ve yapay zekânın sonucu sade dille anlatmasını sağlar.

## Nasıl çalışır

```
.bin ──▶ parser.py (pymavlink DFReader) ──▶ FlightLog (saf domain modeli)
                                               │
                                               ▼
                       checks/ registry ──▶ Check eklentileri (her biri bir konu)
                                               │
                                               ▼
                    orchestrator.diagnose() ──▶ DiagnosisReport (yapısal)
                                               │
                                               ▼
                       server.py (FastMCP, stdio) ──▶ Yapay zekâ bulguları anlatır
```

- **Anahtar ilke:** deterministik motor **tek doğruluk kaynağı**; yapay zekâ sadece açıklar ve kaynak gösterir, *karar vermez*.
- **`FlightLog`** saf, `pymavlink`-bağımsız bir domain modelidir; bu yüzden kontroller ve testler gerçek `.bin` olmadan sentetik loglarla çalışır.
- **Kontroller** bağımsız, registry'e kayıtlı eklentilerdir (`@register_check`). Yeni bir tane eklemek = tek bir yeni modül. Orchestrator, verisi loglanmamış kontrolü zarifçe atlar, araç tipine uymayanı (ör. heli/uçakta motor-dengesi) çalıştırmaz ve hata fırlatan kontrolü izole eder.
- Katalog iki aile: **Uçuş dinamiği** — olaylar/hatalar, EKF sağlığı, titreşim, güç (BAT/CURR), GPS (fix/uydu/HDOP, armed pencerede), pusula, attitude takibi, motor dengesi, RC sinyal kaybı, zamanlama. **Konfigürasyon & kurulum** (çünkü çoğu kaza uçuş anında değil kurulum hatasıdır): riskli parametreler (kapalı arming/failsafe), çelişkili/kapalı/mantıksız değerlerin **parametre denetimi**, kalibrasyon (büyük/sıfır pusula offset'i), yapılandırılmış-ama-sessiz sensörler (kablo/bağlantı arızası), ve firmware'in kendi pre-arm/açılış uyarıları. Artı tavsiye niteliğinde tuning.
- **Araç-farkında** (copter / heli / uçak / rover): çoklu-rotora özel kontroller bir helinin swashplate servolarında ya da bir uçağın kontrol yüzeylerinde çalışmaz. Copter 3.2–4.6, Plane, QuadPlane, Heli ve Rover'ı kapsayan **40 gerçek forum loguyla doğrulandı.**
- Bulgular **doküman-temelli**: her biri ilgili resmi ArduPilot sayfasına link verir; raporlar 4.x için sürüm-özel parametre tanımlarını (`apm.pdef.xml`) yüzeye çıkarır. Sunucu offline/deterministik kalır; bu *işaretçileri* web erişimi olan yapay zekâya verir, o okur ve alıntılar.
- **Fiziksel profil** (`vehicle_profile`) logu airframe gerçeklerine dönüştürür — frame, motor sayısı, batarya hücresi ve özellikle **güç marjı** (hover gazı → itki/ağırlık). Motorlarını sonuna kadar açıp düşen gerçek bir 12 kg'lık quad'da *"underpowered, %69 gazda asılı, sadece %31 pay"* diyor — kazanın asıl sebebi.

## Güvenlik

Bu sürüm **offline ve salt-okunur**. MAVLink bağlantısı yok, arming yok, parametre yazma yok, aktüasyon yok. Tuning çıktısı sadece tavsiyedir. Canlı-araç özellikleri (düzgün bir güvenlik gateway'i ile: SITL-önce, insan-onayı, kill switch) bilinçli olarak kapsam dışı ve ertelenmiş.

## Kaynaklar & varsayımlar

Koddaki her sayısal eşik, enum, varsayılan ve davranışsal iddia otoriter kaynaklara karşı (ArduPilot wiki/firmware, MAVLink, MCP spec) [`docs/SOURCES.md`](docs/SOURCES.md)'de denetlenmiştir — her biri *onaylandı* (atıflı), *heuristik* (bizim ihtiyatlı seçimimiz, en yakın rehber atıflı) veya *tasarım kararı* (bizim severity sınırımız) olarak işaretli. Bu denetim 8 gerçek hatayı bulup düzeltti (ör. yanlış bir heli FRAME_CLASS kümesi, yanlış-numaralı event id'leri, innovation test-ratio'su olmayan bir EKF alanı).

## Geliştirme

Katkılar için: [CONTRIBUTING.md](CONTRIBUTING.md) ve [AGENTS.md](AGENTS.md) (insanlar ve yapay zekâ asistanları için kurallar).

```bash
pip install -e ".[dev]"
ruff check ardupilot_mcp tests        # lint
mypy ardupilot_mcp                    # tip kontrolü
pytest                                # testler
```

Kontroller sentetik in-memory loglarla (`tests/helpers.py`) test edilir; sentetik bir DataFlash yazıcısı (`tests/synth_bin.py`) parser'a round-trip için gerçek, hermetik bir `.bin` verir — repoya büyük ikili fixture girmez.

## Bilinen sınırlamalar

- **Toilet-bowl / pusula-heading** arızaları henüz tespit edilmiyor — sadece alan-büyüklüğü stabilitesi değil, dairesel-pozisyon-drift analizi gerektiriyor (planlı).
- **Gerçekten bozuk bir log** (ör. bit-flip'li mesaj formatı) kısmi-kurtarma yerine net bir hatayla başarısız olur.
- Büyük loglar (3–4 MB / ~100k kayıt) ~12–15 sn'de parse olur; MCP sunucusu parse'ı path+mtime ile cache'ler, yani bu maliyeti sadece ilk araç çağrısı öder.

## Yol haritası

1. **Bu sürüm:** ArduPilot offline `.bin` teşhisi + tuning tavsiyesi + genişletilebilir kontrol çerçevesi.
2. PX4 / ULog desteği (ayrı parser + kurallar; ortak teşhis soyutlaması).
3. Canlı MAVLink bağlantısı (SITL-önce) + tam güvenlik gateway'i.
4. Topluluk kontrol-paylaşımı; onay-kapılı tuning *uygulaması*.

## Lisans

MIT
